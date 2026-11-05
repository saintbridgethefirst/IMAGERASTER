/*
 * Load your own picture as the scan sheet (key I), right inside the twin.
 *
 * Same layout as tools/make_sheet.py: the picture's bottom-left corner sits
 * 45 mm ahead of the start axle on the first row line; the start cross, wheel
 * ticks and a ruler are added; the page (A4 or A3) is chosen to fit. You get:
 *   sheets/<name>.pdf          print this at 100 % ("Actual size")
 *   sheets/<name>_surface.png  the sheet at 1 px/mm (what the twin scans)
 *   sheets/<name>.json         where it sits, and the scan settings for it
 * and the twin switches to it at once (also saved as surface.png/.json, so it
 * is still there next time).
 *
 * The picture can be PNG, JPG, GIF or TGA. You can also pick a sheet that
 * make_sheet.py made (its .json, or its _template.png).
 */

/** Picture's left edge in the robot's frame (mm ahead of the start axle). */
final float SHEET_PICTURE_X0 = 45.0;

/** @brief File dialog callback for the I key (runs on another thread). */
public void sheetImageSelected(File selection) {
  if (selection != null) simulator.pendingSheet = selection;
}

/**
 * @brief Asks how wide the picture will be printed.
 * @return Width in mm, or NaN if cancelled.
 */
float askPrintedWidth(String name, float suggested) {
  String a = javax.swing.JOptionPane.showInputDialog(null,
    "How wide should \"" + name + "\" be when printed (mm)?\n"
    + "The twin and the printable PDF both use this size.", nf(suggested, 0, 1));
  if (a == null) return Float.NaN;
  float v = parseFloat(trim(a));
  return (Float.isNaN(v) || v < 10 || v > 400) ? Float.NaN : v;
}

/** @return Pixels per mm from a PNG pHYs chunk or a JPEG JFIF header, or NaN. */
float imagePxPerMm(byte[] b) {
  if (b == null || b.length < 24) return Float.NaN;
  if ((b[0] & 0xff) == 0x89 && b[1] == 'P') {
    int p = 8;
    while (p + 8 <= b.length) {
      int len = ((b[p] & 0xff) << 24) | ((b[p + 1] & 0xff) << 16) | ((b[p + 2] & 0xff) << 8) | (b[p + 3] & 0xff);
      String type = new String(b, p + 4, 4);
      int d = p + 8;
      if (type.equals("pHYs") && len >= 9 && d + 9 <= b.length && b[d + 8] == 1) {
        long ppu = ((long)(b[d] & 0xff) << 24) | ((b[d + 1] & 0xff) << 16) | ((b[d + 2] & 0xff) << 8) | (b[d + 3] & 0xff);
        return ppu / 1000.0;
      }
      if (type.equals("IDAT") || len < 0) break;
      p = d + len + 4;
    }
  }
  if ((b[0] & 0xff) == 0xff && (b[1] & 0xff) == 0xd8 && (b[3] & 0xff) == 0xe0 && b[6] == 'J' && b[7] == 'F') {
    int units = b[13] & 0xff, xd = ((b[14] & 0xff) << 8) | (b[15] & 0xff);
    if (units == 1 && xd > 0) return xd / 25.4;
    if (units == 2 && xd > 0) return xd / 10.0;
  }
  return Float.NaN;
}

/** @return The JSON stored in a PNG tEXt chunk with this keyword, or null. */
JSONObject pngTextJson(byte[] b, String key) {
  if (b == null || b.length < 8 || (b[0] & 0xff) != 0x89) return null;
  int p = 8;
  while (p + 8 <= b.length) {
    int len = ((b[p] & 0xff) << 24) | ((b[p + 1] & 0xff) << 16) | ((b[p + 2] & 0xff) << 8) | (b[p + 3] & 0xff);
    if (len < 0 || p + 12 + len > b.length) break;
    String type = new String(b, p + 4, 4);
    if (type.equals("tEXt")) {
      String txt = new String(b, p + 8, len, java.nio.charset.StandardCharsets.ISO_8859_1);
      int z = txt.indexOf('\0');
      if (z > 0 && txt.substring(0, z).equals(key)) return parseJSONObject(txt.substring(z + 1));
    }
    if (type.equals("IEND")) break;
    p += 12 + len;
  }
  return null;
}

/** @brief Where a picture goes on the page, and the scan that covers it. */
class SheetLayout_c {
  String name;
  float w, h;            // picture, mm
  float pitch;
  int rows;
  float rowLen;
  String pageName;
  float PW, PH;          // page, mm
  float left, bottom;    // world position (mm) of the page's bottom-left corner

  float pageX(float wx) { return wx - left; }
  float pageY(float wy) { return (bottom + PH) - wy; }

  JSONObject toJson() {
    JSONObject j = new JSONObject();
    j.setString("sheet", name);
    j.setString("page", pageName);
    j.setFloat("offset_x_mm", left + PW / 2);
    j.setFloat("offset_y_mm", bottom + PH / 2);
    JSONObject s = new JSONObject();
    s.setInt("pattern", 0);
    s.setInt("rows", rows);
    s.setFloat("row_len", rowLen);
    s.setFloat("pitch", pitch);
    s.setInt("side", 1);
    j.setJSONObject("settings", s);
    return j;
  }
}

/** @return The layout for a w x h mm picture, or null if it doesn't fit on A3. */
SheetLayout_c layoutSheet(String name, float w, float h, float pitch) {
  SheetLayout_c L = new SheetLayout_c();
  L.name = name;
  L.w = w;
  L.h = h;
  L.pitch = pitch;
  L.rows = max(1, ceil((h - 9) / pitch) + 1);       // middle sensors reach 9 mm past the rows
  L.rowLen = ceil(w + 2 * (SHEET_PICTURE_X0 - 41)); // DN3 is 41 mm ahead of the axle
  float topRows = (L.rows - 1) * pitch;
  float cx0 = -58, cx1 = SHEET_PICTURE_X0 + w + 15;
  float cy0 = -60, cy1 = max(h, topRows + 27) + 8;
  float needW = cx1 - cx0, needH = cy1 - cy0;
  String[] names = {"A4 landscape", "A4 portrait", "A3 landscape", "A3 portrait"};
  float[][] sizes = {{297, 210}, {210, 297}, {420, 297}, {297, 420}};
  for (int i = 0; i < names.length; i++) {
    if (needW <= sizes[i][0] - 10 && needH <= sizes[i][1] - 10) {
      L.pageName = names[i];
      L.PW = sizes[i][0];
      L.PH = sizes[i][1];
      L.left = cx0 - (L.PW - needW) / 2;
      L.bottom = cy0 - (L.PH - needH) / 2;
      return L;
    }
  }
  return null;
}

/**
 * @brief Draws the sheet in page millimetres (origin top-left) on g: the
 * picture, the start cross, wheel ticks, arrow, ruler and (for print) labels.
 */
void drawSheet(PGraphics g, PImage pic, SheetLayout_c L, boolean labels) {
  g.background(255);
  g.imageMode(CORNER);
  g.image(pic, L.pageX(SHEET_PICTURE_X0), L.pageY(L.h), L.w, L.h);
  g.noFill();
  // Start: axle cross, wheel ticks (41 mm either side), arrow.
  g.stroke(80);
  g.strokeWeight(0.4);
  g.line(L.pageX(-6), L.pageY(0), L.pageX(6), L.pageY(0));
  g.line(L.pageX(0), L.pageY(-6), L.pageX(0), L.pageY(6));
  g.stroke(150, 160, 170);
  g.strokeWeight(0.8);
  g.line(L.pageX(-9), L.pageY(41.25), L.pageX(-1), L.pageY(41.25));
  g.line(L.pageX(-9), L.pageY(-41.25), L.pageX(-1), L.pageY(-41.25));
  g.stroke(110);
  g.strokeWeight(0.6);
  g.line(L.pageX(-46), L.pageY(-8), L.pageX(-14), L.pageY(-8));
  g.line(L.pageX(-14), L.pageY(-8), L.pageX(-18), L.pageY(-5.5));
  g.line(L.pageX(-14), L.pageY(-8), L.pageX(-18), L.pageY(-10.5));
  // Ruler from the start axle, below the scan.
  float ry = -36, rlen = SHEET_PICTURE_X0 + L.w + 12;
  g.stroke(60);
  g.strokeWeight(0.3);
  g.line(L.pageX(0), L.pageY(ry), L.pageX(rlen), L.pageY(ry));
  for (int mm = 0; mm <= (int) rlen; mm++) {
    float tick = mm % 10 == 0 ? 5 : (mm % 5 == 0 ? 3.5 : 2);
    g.strokeWeight(mm % 10 == 0 ? 0.3 : 0.15);
    g.line(L.pageX(mm), L.pageY(ry), L.pageX(mm), L.pageY(ry - tick));
  }
  if (!labels) return;
  g.fill(70);
  g.noStroke();
  g.textAlign(LEFT, BASELINE);
  g.textSize(2.6);
  g.text("START: wheel-axle centre on +, facing the arrow;", L.pageX(-56), L.pageY(-14));
  g.text("grey ticks under the wheels. Next row: robot's LEFT (up).", L.pageX(-56), L.pageY(-18));
  g.textAlign(CENTER, BASELINE);
  g.textSize(2.4);
  for (int mm = 0; mm <= (int) rlen; mm += 50) g.text(str(mm), L.pageX(mm), L.pageY(ry - 8));
  g.textAlign(LEFT, BASELINE);
  g.textSize(3.4);
  g.text(L.name, L.pageX(-56), L.pageY(-51));
  g.textSize(2.6);
  g.text("Dashboard: Rows " + L.rows + ", Row length " + round(L.rowLen) + " mm, Row pitch " + nf(L.pitch, 0, 1)
    + " mm, next row on the left. Print at 100 % (Actual size) on " + L.pageName.split(" ")[0] + ".",
    L.pageX(-56), L.pageY(-56));
}

/** @return The sheet at 1 pixel per mm, for the twin to scan. */
PImage renderSheetSurface(PImage pic, SheetLayout_c L) {
  PGraphics g = createGraphics(ceil(L.PW), ceil(L.PH));
  g.beginDraw();
  drawSheet(g, pic, L, false);
  g.endDraw();
  return g.get();
}

/** @brief Writes the printable PDF of the sheet (page size in mm). */
void writeSheetPdf(PImage pic, SheetLayout_c L, String path) {
  float ptPerMm = 72 / 25.4;
  PGraphics pdf = createGraphics(round(L.PW * ptPerMm), round(L.PH * ptPerMm), PDF, path);
  pdf.beginDraw();
  pdf.textFont(createFont("SansSerif", 12));
  pdf.scale(ptPerMm);
  drawSheet(pdf, pic, L, true);
  pdf.dispose();
  pdf.endDraw();
}
