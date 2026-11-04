/**
 * @brief Rebuilds an image from raster-scan samples, the way the dashboard
 * does (simplified): each reading is placed where the robot BELIEVED that
 * sensor was, scaled per sensor to 0..1 darkness, and spread over nearby
 * 1 mm cells. Comparing the rebuild with the true surface measures how well
 * the scan worked; comparing sim and real rebuilds measures the twin.
 */
class RasterImage_c {
  String title;
  /** World position (mm) of the grid's bottom-left corner, and size in cells. */
  float x0, y0;
  int w = 1, h = 1;
  /** Target area for the scores: where the middle sensors should cover. */
  float tx0, ty0, tx1, ty1;

  // Every reading: world x, world y, raw value, sensor index.
  FloatList hitX = new FloatList();
  FloatList hitY = new FloatList();
  FloatList hitV = new FloatList();
  IntList hitS = new IntList();

  float[] darkness = new float[1];
  float[] weight = new float[1];
  PImage image;
  boolean dirty = false;

  float coverage = 0;      // fraction of target cells with data
  float maeVsTruth = -1;   // mean |darkness error| over covered target cells (0..1)

  RasterImage_c(String titleIn) {
    title = titleIn;
    image = createImage(1, 1, RGB);
  }

  /** @brief Sizes the grid for a scan configuration and sensor layout. */
  void setArea(RasterConfig_c cfg, ModelParameters_c layout) {
    float aheadMin = 1e9, aheadMax = -1e9, reach = 0;
    for (int i = 0; i < 5; i++) {
      aheadMin = min(aheadMin, layout.sensorAheadMm[i]);
      aheadMax = max(aheadMax, layout.sensorAheadMm[i]);
      reach = max(reach, abs(layout.sensorLeftMm[i]));
    }
    float yEdge = (cfg.rows - 1) * cfg.row_pitch_mm * cfg.side;
    float xa = aheadMin - 15;
    float xb = cfg.row_length_mm + aheadMax + 15;
    float ya = min(0, yEdge) - reach - 10;
    float yb = max(0, yEdge) + reach + 10;
    x0 = floor(xa);
    y0 = floor(ya);
    w = max(1, ceil(xb - x0));
    h = max(1, ceil(yb - y0));
    float lead = layout.sensorAheadMm[2];
    tx0 = lead;
    tx1 = cfg.row_length_mm + lead;
    ty0 = min(0, yEdge) - 9;
    ty1 = max(0, yEdge) + 9;
    darkness = new float[w * h];
    weight = new float[w * h];
    image = createImage(w, h, RGB);
    clear();
  }

  void clear() {
    hitX.clear(); hitY.clear(); hitV.clear(); hitS.clear();
    java.util.Arrays.fill(darkness, 0);
    java.util.Arrays.fill(weight, 0);
    coverage = 0;
    maeVsTruth = -1;
    dirty = true;
  }

  int count() { return hitV.size() / 5; }

  /** @brief Adds the five readings of one sample at a (believed) pose. */
  void addReading(float x, float y, float theta, float[] dn, ModelParameters_c layout) {
    float c = cos(theta), s = sin(theta);
    for (int i = 0; i < 5; i++) {
      float a = layout.sensorAheadMm[i], l = layout.sensorLeftMm[i];
      hitX.append(x + a * c - l * s);
      hitY.append(y + a * s + l * c);
      hitV.append(dn[i]);
      hitS.append(i);
    }
    dirty = true;
  }

  /**
   * @brief Recomputes the image and the scores.
   * @param surface The true surface (for the score), or null.
   */
  void rebuild(PImage surface, float offX, float offY, float offImage) {
    if (!dirty) return;
    dirty = false;
    int n = hitV.size();
    java.util.Arrays.fill(darkness, 0);
    java.util.Arrays.fill(weight, 0);

    // Per-sensor scaling: 2nd percentile = white, 98th = black.
    float[] lo = new float[5], hi = new float[5];
    for (int sIdx = 0; sIdx < 5; sIdx++) {
      FloatList v = new FloatList();
      for (int k = 0; k < n; k++) if (hitS.get(k) == sIdx) v.append(hitV.get(k));
      if (v.size() < 10) { lo[sIdx] = 0; hi[sIdx] = 1; continue; }
      v.sort();
      lo[sIdx] = v.get(floor(0.02 * (v.size() - 1)));
      hi[sIdx] = v.get(ceil(0.98 * (v.size() - 1)));
      if (hi[sIdx] - lo[sIdx] < 50) hi[sIdx] = lo[sIdx] + 50;  // a sensor that saw no contrast
    }

    // Splat each reading over nearby cells (Gaussian, sigma 1 mm).
    for (int k = 0; k < n; k++) {
      int sIdx = hitS.get(k);
      float d = constrain((hitV.get(k) - lo[sIdx]) / (hi[sIdx] - lo[sIdx]), 0, 1);
      float gx = hitX.get(k) - x0 - 0.5, gy = hitY.get(k) - y0 - 0.5;
      int cx = round(gx), cy = round(gy);
      for (int j = cy - 2; j <= cy + 2; j++) {
        if (j < 0 || j >= h) continue;
        for (int i = cx - 2; i <= cx + 2; i++) {
          if (i < 0 || i >= w) continue;
          float dx = i - gx, dy = j - gy;
          float wt = exp(-(dx * dx + dy * dy) / 2.0);
          int idx = j * w + i;
          darkness[idx] += wt * d;
          weight[idx] += wt;
        }
      }
    }

    // Image (row 0 = top = largest y) and scores.
    image.loadPixels();
    int covered = 0, targetCells = 0;
    float errSum = 0;
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        int idx = j * w + i;
        int py = h - 1 - j;
        boolean seen = weight[idx] > 0.05;
        float dk = seen ? darkness[idx] / weight[idx] : 0;
        if (seen) {
          image.pixels[py * w + i] = color(255 * (1 - dk));
        } else {
          boolean check = ((i / 4) + (j / 4)) % 2 == 0;
          image.pixels[py * w + i] = check ? color(205, 215, 230) : color(225, 232, 242);
        }
        float wx = x0 + i + 0.5, wy = y0 + j + 0.5;
        if (wx >= tx0 && wx <= tx1 && wy >= ty0 && wy <= ty1) {
          targetCells++;
          if (seen) {
            covered++;
            if (surface != null) {
              float truth = 1 - surfaceReflectance(surface, offX, offY, wx, wy, offImage);
              errSum += abs(dk - truth);
            }
          }
        }
      }
    }
    image.updatePixels();
    coverage = targetCells > 0 ? covered / (float) targetCells : 0;
    maeVsTruth = (surface != null && covered > 0) ? errSum / covered : -1;
  }

  /** @return Darkness 0..1 at a cell, or -1 if nothing was seen there. */
  float darknessAt(int i, int j) {
    int idx = j * w + i;
    return weight[idx] > 0.05 ? darkness[idx] / weight[idx] : -1;
  }

  /** @brief Draws the image to fit a box, keeping the aspect ratio. */
  void draw(float bx, float by, float bw, float bh) {
    float s = min(bw / w, bh / h);
    float dw = w * s, dh = h * s;
    pushStyle();
    imageMode(CORNER);
    image(image, bx + (bw - dw) / 2, by, dw, dh);
    noFill();
    stroke(120);
    rect(bx + (bw - dw) / 2, by, dw, dh);
    popStyle();
  }
}

/** @return Mean |darkness difference| where both rebuilds have data, or -1. */
float compareRebuilds(RasterImage_c a, RasterImage_c b) {
  if (a.w != b.w || a.h != b.h) return -1;
  float sum = 0;
  int n = 0;
  for (int j = 0; j < a.h; j++) {
    for (int i = 0; i < a.w; i++) {
      float wx = a.x0 + i + 0.5, wy = a.y0 + j + 0.5;
      if (wx < a.tx0 || wx > a.tx1 || wy < a.ty0 || wy > a.ty1) continue;
      float da = a.darknessAt(i, j), db = b.darknessAt(i, j);
      if (da < 0 || db < 0) continue;
      sum += abs(da - db);
      n++;
    }
  }
  return n > 0 ? sum / n : -1;
}

/** @brief The true surface on the same grid, for side-by-side display. */
PImage truthImage(RasterImage_c grid, PImage surface, float offX, float offY, float offImage) {
  PImage img = createImage(grid.w, grid.h, RGB);
  img.loadPixels();
  for (int j = 0; j < grid.h; j++) {
    for (int i = 0; i < grid.w; i++) {
      float r = surfaceReflectance(surface, offX, offY, grid.x0 + i + 0.5, grid.y0 + j + 0.5, offImage);
      img.pixels[(grid.h - 1 - j) * grid.w + i] = color(255 * r);
    }
  }
  img.updatePixels();
  return img;
}

/**
 * @brief Writes samples in the dashboard's CSV format (the dashboard can load
 * it), plus the true pose for simulated runs.
 */
void writeScanCsv(String path, ArrayList<ScanSample_c> samples, boolean withTruth, String comment) {
  PrintWriter out = createWriter(path);
  out.println("# " + comment);
  out.println("seq,t_ms,x_mm,y_mm,theta_rad,dn1,dn2,dn3,dn4,dn5,state,row"
    + (withTruth ? ",true_x_mm,true_y_mm,true_theta_rad" : ""));
  long t0 = samples.size() > 0 ? samples.get(0).t_ms : 0;
  for (int k = 0; k < samples.size(); k++) {
    ScanSample_c s = samples.get(k);
    String line = k + "," + (s.t_ms - t0) + "," + fmtUS(s.x, 2) + "," + fmtUS(s.y, 2) + ","
      + fmtUS(s.theta, 5);
    for (int i = 0; i < 5; i++) line += "," + round(s.dn[i]);
    line += "," + s.state + "," + s.row;
    if (withTruth) line += "," + fmtUS(s.true_x, 2) + "," + fmtUS(s.true_y, 2) + "," + fmtUS(s.true_theta, 5);
    out.println(line);
  }
  out.flush();
  out.close();
}

/** @brief Appends one line to runs/summary.csv (writes the header first time). */
void appendSummary(String path, String header, String line) {
  try {
    File f = new File(path);
    boolean fresh = !f.exists();
    if (f.getParentFile() != null) f.getParentFile().mkdirs();
    java.io.FileWriter fw = new java.io.FileWriter(f, true);
    if (fresh) fw.write(header + "\n");
    fw.write(line + "\n");
    fw.close();
  }
  catch (Exception e) {
    println("Could not write " + path + ": " + e.getMessage());
  }
}

/** @return A number with a dot as the decimal mark, whatever the computer's language. */
String fmtUS(float v, int digits) {
  return String.format(java.util.Locale.US, "%." + digits + "f", v);
}
