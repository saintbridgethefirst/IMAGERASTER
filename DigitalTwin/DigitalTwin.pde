import processing.net.*;
import processing.core.PApplet;
import java.net.*;

/*
 * ImageRaster Digital Twin, built on the SEMTM0043 CW2026 Digital Twin
 * (github.com/paulodowd/RoboticsScienceAndSystems/DigitalTwin).
 *
 * The robot scans surface.png (the printed sheet at 1 pixel per mm) with the
 * same raster controller as the StampC3 (Controller.pde = Controller.cpp).
 * Put your Phase 1 measurements in ModelParameters.pde.
 *
 * Keys: S scan, X stop, N 10 repeats, F fast, P nominal/calibrated model,
 *       L load a robot scan CSV (the dashboard's Download CSV),
 *       O lay the robot's or the twin's rebuild over the sheet,
 *       C copy the robot's scan settings, M sandbox/live comparison,
 *       R image rebuilds, G graphs, T track, +/- zoom, arrows pan.
 * Results (CSV, PNG, runs/summary.csv) are written to the runs/ folder.
 */

/** @brief Initial application-window width in screen pixels. */
final int WINDOW_WIDTH = 1200;
/** @brief Initial application-window height in screen pixels. */
final int WINDOW_HEIGHT = 800;
/**
 * @brief World position (mm) of the CENTRE of surface.png, used only when
 * there is no surface.json (tools/make_sheet.py writes one for every sheet,
 * with this position and the scan settings). The robot's axle
 * starts every scan at (0, 0) facing +x. For test_sheet_A3 (420 x 297 mm),
 * whose start cross is 125 mm from the left edge and 90 mm from the bottom,
 * the centre is at (210 - 125, 148.5 - 90) = (85, 58.5).
 * For your own sheet: centre = (width/2 - start_from_left, height/2 - start_from_bottom).
 */
final float SURFACE_OFFSET_X_MM = 85.0;
final float SURFACE_OFFSET_Y_MM = 58.5;

/** @brief Application-level owner of the simulated robot and UI. */
Simulator simulator;

void settings() {
  size(WINDOW_WIDTH, WINDOW_HEIGHT);
}

void setup() {
  surface.setResizable(true);
  surface.setTitle("ImageRaster Digital Twin");
  simulator = new Simulator(this, SURFACE_OFFSET_X_MM, SURFACE_OFFSET_Y_MM);
}

void draw() {
  simulator.update();
  simulator.draw();
}

void keyPressed() {
  simulator.handleKey(key, keyCode);
}

/** @brief Called by the L key's file dialog (on another thread). */
public void robotCsvSelected(File selection) {
  if (selection != null) simulator.pendingCsv = selection;
}
