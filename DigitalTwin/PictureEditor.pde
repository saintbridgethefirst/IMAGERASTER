/**
 * @brief Rotate and crop a picture before it becomes the scan sheet (opened by
 * the I key; E reopens the last picture).
 *
 *   drag            draw the crop (drag inside it to move, drag a corner to resize)
 *   [ / ]           rotate 1 degree anticlockwise / clockwise
 *   { / }           rotate 0.1 degree (shift + [ / ])
 *   R               rotate 90 degrees
 *   A               auto-crop to the drawing (trims white margins)
 *   F               no crop (the whole picture)
 *   Enter           use it        Esc  cancel
 */
class PictureEditor_c {
  PImage original;          // as loaded (scaled down if very large)
  PImage rotated;           // original rotated by angle, on white
  float angle = 0;          // degrees, positive = clockwise on screen
  float cx0, cy0, cx1, cy1; // crop, in rotated-image pixels
  String fileName = "";
  String base = "";
  float pxPerMm = Float.NaN; // of the working copy, if the file said
  // screen mapping: screen = v + pixel * vs
  float vx, vy, vs = 1;
  int dragMode = 0;         // 0 none, 1-4 corners, 5 move, 6 new rectangle
  float grabX, grabY;

  boolean hasPicture() { return original != null; }

  void open(PImage pic, String fileNameIn, String baseIn, float ppm) {
    original = pic.copy();
    float big = max(original.width, original.height);
    float k = big > 2400 ? 2400 / big : 1;
    if (k < 1) original.resize(round(original.width * k), round(original.height * k));
    pxPerMm = Float.isNaN(ppm) ? Float.NaN : ppm * k;
    fileName = fileNameIn;
    base = baseIn;
    angle = 0;
    rebuild(true);
  }

  /** @brief Re-renders the rotation; keeps the crop in proportion unless reset. */
  void rebuild(boolean resetCrop) {
    float fx0 = 0, fy0 = 0, fx1 = 1, fy1 = 1;
    if (!resetCrop && rotated != null) {
      fx0 = cx0 / rotated.width; fx1 = cx1 / rotated.width;
      fy0 = cy0 / rotated.height; fy1 = cy1 / rotated.height;
    }
    float a = radians(angle);
    int w = original.width, h = original.height;
    int bw = max(1, ceil(abs(w * cos(a)) + abs(h * sin(a))));
    int bh = max(1, ceil(abs(w * sin(a)) + abs(h * cos(a))));
    PGraphics g = createGraphics(bw, bh);
    g.beginDraw();
    g.background(255);
    g.imageMode(CENTER);
    g.translate(bw / 2.0, bh / 2.0);
    g.rotate(a);
    g.image(original, 0, 0);
    g.endDraw();
    rotated = g.get();
    rotated.loadPixels();
    cx0 = fx0 * bw; cx1 = fx1 * bw; cy0 = fy0 * bh; cy1 = fy1 * bh;
  }

  void rotateBy(float degrees) {
    angle = ((angle + degrees) % 360 + 360) % 360;
    if (angle > 180) angle -= 360;
    angle = round(angle * 10) / 10.0;
    rebuild(abs(degrees) >= 90);
  }

  /** @brief Crops to the drawing: everything darker than near-white, plus a margin. */
  void autoCrop() {
    int w = rotated.width, h = rotated.height;
    int x0 = w, y0 = h, x1 = -1, y1 = -1;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        int c = rotated.pixels[y * w + x];
        float lum = 0.299 * ((c >> 16) & 0xff) + 0.587 * ((c >> 8) & 0xff) + 0.114 * (c & 0xff);
        if (lum < 215) {
          if (x < x0) x0 = x;
          if (x > x1) x1 = x;
          if (y < y0) y0 = y;
          if (y > y1) y1 = y;
        }
      }
    }
    if (x1 < 0) { cropAll(); return; }
    float m = 0.02 * max(w, h);
    cx0 = max(0, x0 - m); cy0 = max(0, y0 - m);
    cx1 = min(w, x1 + 1 + m); cy1 = min(h, y1 + 1 + m);
  }

  void cropAll() { cx0 = 0; cy0 = 0; cx1 = rotated.width; cy1 = rotated.height; }

  /** @return The rotated, cropped picture. */
  PImage result() {
    int x = round(min(cx0, cx1)), y = round(min(cy0, cy1));
    int w = max(1, round(abs(cx1 - cx0))), h = max(1, round(abs(cy1 - cy0)));
    return rotated.get(x, y, w, h);
  }

  // ---- drawing ------------------------------------------------------------

  void draw() {
    background(38, 42, 50);
    float top = 96, margin = 24;
    vs = min((width - 2 * margin) / rotated.width, (height - top - margin) / rotated.height);
    vx = (width - rotated.width * vs) / 2;
    vy = top + (height - top - margin - rotated.height * vs) / 2;
    imageMode(CORNER);
    image(rotated, vx, vy, rotated.width * vs, rotated.height * vs);

    float sx0 = vx + min(cx0, cx1) * vs, sx1 = vx + max(cx0, cx1) * vs;
    float sy0 = vy + min(cy0, cy1) * vs, sy1 = vy + max(cy0, cy1) * vs;
    float ix0 = vx, ix1 = vx + rotated.width * vs, iy0 = vy, iy1 = vy + rotated.height * vs;
    noStroke();
    fill(0, 150);
    rectMode(CORNERS);
    rect(ix0, iy0, ix1, sy0);
    rect(ix0, sy1, ix1, iy1);
    rect(ix0, sy0, sx0, sy1);
    rect(sx1, sy0, ix1, sy1);
    noFill();
    stroke(255, 200, 40);
    strokeWeight(2);
    rect(sx0, sy0, sx1, sy1);
    // rule-of-thirds lines help straighten a photo
    stroke(255, 200, 40, 90);
    strokeWeight(1);
    for (int i = 1; i < 3; i++) {
      line(sx0 + (sx1 - sx0) * i / 3, sy0, sx0 + (sx1 - sx0) * i / 3, sy1);
      line(sx0, sy0 + (sy1 - sy0) * i / 3, sx1, sy0 + (sy1 - sy0) * i / 3);
    }
    fill(255, 200, 40);
    noStroke();
    rectMode(CENTER);
    float[][] corners = {{sx0, sy0}, {sx1, sy0}, {sx1, sy1}, {sx0, sy1}};
    for (float[] c : corners) rect(c[0], c[1], 10, 10);
    rectMode(CORNER);

    int cw = round(abs(cx1 - cx0)), ch = round(abs(cy1 - cy0));
    String size = cw + " x " + ch + " px";
    if (!Float.isNaN(pxPerMm) && pxPerMm > 0) size += " (" + nf(cw / pxPerMm, 0, 0) + " x " + nf(ch / pxPerMm, 0, 0) + " mm at the file's size)";
    fill(235);
    textSize(16);
    textAlign(LEFT, BASELINE);
    text("Rotate and crop \"" + fileName + "\"   angle " + nf(angle, 0, 1) + " deg   crop " + size, margin, 30);
    textSize(13);
    fill(200);
    text("Drag to draw the crop (inside = move, corner = resize)    [ ] rotate 1 deg    shift+[ ] 0.1 deg    R 90 deg    A auto-crop    F whole picture",
      margin, 56);
    text("Enter: use it as the sheet (asks the printed width next)    Esc: cancel.    Rows run left to right across the picture as shown here.",
      margin, 76);
  }

  // ---- mouse ----------------------------------------------------------------

  float toImgX(float mx) { return (mx - vx) / vs; }
  float toImgY(float my) { return (my - vy) / vs; }

  void mousePressed(float mx, float my) {
    float sx0 = vx + cx0 * vs, sx1 = vx + cx1 * vs, sy0 = vy + cy0 * vs, sy1 = vy + cy1 * vs;
    float[][] corners = {{sx0, sy0}, {sx1, sy0}, {sx1, sy1}, {sx0, sy1}};
    dragMode = 0;
    for (int i = 0; i < 4; i++) {
      if (dist(mx, my, corners[i][0], corners[i][1]) < 12) { dragMode = i + 1; return; }
    }
    float ix = toImgX(mx), iy = toImgY(my);
    if (ix > min(cx0, cx1) && ix < max(cx0, cx1) && iy > min(cy0, cy1) && iy < max(cy0, cy1)) {
      dragMode = 5;
      grabX = ix;
      grabY = iy;
      return;
    }
    dragMode = 6;
    cx0 = cx1 = constrain(ix, 0, rotated.width);
    cy0 = cy1 = constrain(iy, 0, rotated.height);
  }

  void mouseDragged(float mx, float my) {
    float ix = constrain(toImgX(mx), 0, rotated.width), iy = constrain(toImgY(my), 0, rotated.height);
    if (dragMode == 1) { cx0 = ix; cy0 = iy; }
    if (dragMode == 2) { cx1 = ix; cy0 = iy; }
    if (dragMode == 3) { cx1 = ix; cy1 = iy; }
    if (dragMode == 4) { cx0 = ix; cy1 = iy; }
    if (dragMode == 6) { cx1 = ix; cy1 = iy; }
    if (dragMode == 5) {
      float dx = toImgX(mx) - grabX, dy = toImgY(my) - grabY;
      float w = cx1 - cx0, h = cy1 - cy0;
      float nx0 = constrain(cx0 + dx, 0, rotated.width - w), ny0 = constrain(cy0 + dy, 0, rotated.height - h);
      grabX += nx0 - cx0;
      grabY += ny0 - cy0;
      cx0 = nx0; cy0 = ny0; cx1 = nx0 + w; cy1 = ny0 + h;
    }
  }

  void mouseReleased() {
    float x0 = min(cx0, cx1), x1 = max(cx0, cx1), y0 = min(cy0, cy1), y1 = max(cy0, cy1);
    if (x1 - x0 < 8 || y1 - y0 < 8) { cropAll(); } else { cx0 = x0; cx1 = x1; cy0 = y0; cy1 = y1; }
    dragMode = 0;
  }

  /** @return True if the key was an editor key (rotation, crop). */
  boolean handleKey(char k) {
    if (k == ']') { rotateBy(1); return true; }
    if (k == '[') { rotateBy(-1); return true; }
    if (k == '}') { rotateBy(0.1); return true; }
    if (k == '{') { rotateBy(-0.1); return true; }
    if (k == 'r' || k == 'R') { rotateBy(90); return true; }
    if (k == 'a' || k == 'A') { autoCrop(); return true; }
    if (k == 'f' || k == 'F') { cropAll(); return true; }
    return false;
  }
}
