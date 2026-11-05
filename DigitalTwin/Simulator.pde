/**
 * @brief Owns the application: timing, robot model, raster comparison, view, TCP.
 * @details World positions use millimetres, with +x forward/right and +y
 * upward. The robot's wheel-axle centre starts every scan at (0, 0) facing +x,
 * like the real robot, whose odometry is zeroed at the start of each scan.
 * Rendering may run at a variable frame rate; model/controller updates use a
 * fixed 10 ms interval.
 *
 * Changes from the template Simulator: raster controller, two physical models
 * (P), scan start/stop/repeat keys, fast-forward, image rebuilds of the twin
 * and of the real robot (from its TCP telemetry), CSV/PNG/summary output in
 * runs/, and the robot's scan settings copied from its dashboard.
 */
class Simulator {
  final int graphWindowMs = 5000;
  final int graphSamplePeriodMs = 20;
  final int simulationUpdateIntervalMs = 10;
  final int maximumCatchUpSteps = 10;
  /** Model steps per frame in fast-forward (sandbox only). */
  final int fastStepsPerFrame = 200;
  final String tcpHost = "192.168.4.1";
  final int tcpPort = 80;
  /** The robot's dashboard: its scan settings are copied from here. */
  final String robotConfigUrl = "http://192.168.4.1:8080/config";
  final int tcpRetryPeriodMs = 2000;
  final int tcpConnectTimeoutMs = 250;
  final float minimumZoom = 0.25;
  final float maximumZoom = 6.0;
  final int SANDBOX = 0, LIVE_COMPARISON = 1;

  float surfaceOffsetX;
  float surfaceOffsetY;
  String sheetName = "surface.png";

  PImage surfaceImage;
  PImage surfaceViewportCache;
  int surfaceCacheX = -1;
  int surfaceCacheY = -1;
  int surfaceCacheWidth = 0;
  int surfaceCacheHeight = 0;
  PApplet applet;

  ModelParameters_c nominal = nominalModel();
  ModelParameters_c calibrated = calibratedModel();
  ModelParameters_c activeModel = calibrated;
  Robot_c robot;
  Controller_c controller;
  TimeSeriesGraph motorGraph;
  TimeSeriesGraph sensorGraph;
  TimeSeriesGraph encoderGraph;

  // ---- Raster comparison --------------------------------------------------
  RasterImage_c rasterSim = new RasterImage_c("Twin rebuild");
  RasterImage_c rasterReal = new RasterImage_c("Robot rebuild");
  PImage truth;
  boolean showRaster = true;
  boolean fastForward = false;
  int repeatsRemaining = 0;
  int simLogged = 0;
  boolean simRunPending = false;
  boolean simDoneForLive = false, realDoneForLive = false;
  long simRunSeed = 0;
  String runStamp = "";
  int runCounter = 0;
  int lastRasterRebuildMs = 0;
  ArrayList<PVector> simTrail = new ArrayList<PVector>();
  ArrayList<PVector> realTrail = new ArrayList<PVector>();
  ArrayList<ScanSample_c> realSamples = new ArrayList<ScanSample_c>();
  float simVsReal = -1;
  String simResult = "";
  String realResult = "";
  String savedMessage = "Results are saved in the sketch's runs/ folder.";

  // ---- Robot settings (fetched from the dashboard in a background thread) ---
  // ---- A saved robot scan, loaded with L (dashboard "Download CSV") ---------
  volatile File pendingCsv = null;
  volatile File pendingSheet = null;
  PictureEditor_c editor = new PictureEditor_c();
  boolean editing = false;
  boolean realFromFile = false;
  /** O cycles what is laid over the sheet: 0 nothing, 1 robot rebuild, 2 twin rebuild. */
  int overlay = 0;
  String realSource = "";

  volatile JSONObject fetchedConfig = null;
  volatile boolean configFetchInProgress = false;
  volatile String configFetchError = null;
  int lastConfigFetchMs = -100000;
  String configStatus = "Scan settings: the twin's defaults (C copies the robot's)";

  boolean showTelemetry = false;
  boolean trackRobot = false;
  int simulatorMode = SANDBOX;
  boolean liveArmed = false;
  boolean liveRunning = false;
  int previousTcpSignal = 0;
  long liveStartTcpMs = -1;
  long liveStartSimulationMs = -1;
  long lastTcpTimestampMs = -1;
  TCPTelemetry_c armedPose = new TCPTelemetry_c();
  TCPTelemetry_c ghost = new TCPTelemetry_c();
  ArrayList<TCPTelemetry_c> tcpQueue = new ArrayList<TCPTelemetry_c>();
  float zoom = 2.0;
  float viewCentreX = 110.0;
  float viewCentreY = 40.0;

  int lastGraphSampleMs = 0;
  int lastSimulationUpdateMs = 0;
  int simulationAccumulatorMs = 0;
  int lastTcpAttemptMs = -tcpRetryPeriodMs;

  Client telemetryClient;
  TCPTelemetry_c tcpTelemetry = new TCPTelemetry_c();
  String tcpStatus = "TCP: not connected";
  String lastTcpLine = "";
  volatile boolean tcpAttemptInProgress = false;
  volatile boolean tcpAttemptFailed = false;
  volatile Socket pendingTcpSocket = null;

  Simulator(PApplet appletIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    applet = appletIn;
    // surface.json (written by tools/make_sheet.py) says where the sheet sits
    // and which scan settings cover it. Without it, use DigitalTwin.pde's values.
    JSONObject sheet = loadSheetInfo();
    surfaceOffsetX = (sheet != null && sheet.hasKey("offset_x_mm")) ? sheet.getFloat("offset_x_mm") : surfaceOffsetXIn;
    surfaceOffsetY = (sheet != null && sheet.hasKey("offset_y_mm")) ? sheet.getFloat("offset_y_mm") : surfaceOffsetYIn;
    ellipseMode(RADIUS);
    strokeJoin(ROUND);
    surfaceImage = loadSurfaceImage();
    surfaceImage.loadPixels();
    robot = new Robot_c(activeModel, surfaceImage, surfaceOffsetX, surfaceOffsetY);
    controller = new Controller_c();
    if (sheet != null && sheet.hasKey("settings") && controller.applyRobotConfig(sheet.getJSONObject("settings"))) {
      configStatus = "Sheet \"" + sheet.getString("sheet", "surface.png") + "\": " + controller.config.describe()
        + " (C copies the robot's)";
    }
    if (sheet != null) sheetName = sheet.getString("sheet", "surface.png");
    frameView();
    lastSimulationUpdateMs = millis();

    motorGraph = new TimeSeriesGraph("Motor PWM", graphWindowMs, true);
    motorGraph.addSeries("Sim L", color(220, 70, 70));
    motorGraph.addSeries("Sim R", color(70, 100, 220));
    motorGraph.addSeries("TCP L", color(145, 60, 190));
    motorGraph.addSeries("TCP R", color(20, 150, 150));
    sensorGraph = new TimeSeriesGraph("Surface sensors", graphWindowMs, false);
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("Sim " + (i + 1), sensorColour(i));
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("TCP " + (i + 1), tcpSensorColour(i));
    encoderGraph = new TimeSeriesGraph("Encoder counts", graphWindowMs, false);
    encoderGraph.addSeries("Sim L", color(220, 70, 70));
    encoderGraph.addSeries("Sim R", color(70, 100, 220));
    encoderGraph.addSeries("TCP L", color(145, 60, 190));
    encoderGraph.addSeries("TCP R", color(20, 150, 150));

    rasterReal.setArea(controller.config, calibrated);
    beginSimRun(); // Sandbox starts a scan immediately, like the template.
  }

  // ======================================================================
  // Per-frame update
  // ======================================================================

  void update() {
    updateTcpConnection();
    if (editing) return;   // the model waits while you rotate/crop a picture
    applyFetchedConfig();
    if (pendingSheet != null) {
      File f = pendingSheet;
      pendingSheet = null;
      loadSheetFile(f);
    }
    if (pendingCsv != null) {
      File f = pendingCsv;
      pendingCsv = null;
      loadRobotCsv(f);
    }
    updateRobotModel();
    collectSimSamples();
    checkSimRunEnd();
    if (millis() - lastRasterRebuildMs > 400) {
      lastRasterRebuildMs = millis();
      rasterSim.rebuild(surfaceImage, surfaceOffsetX, surfaceOffsetY, activeModel.offImageReflectance);
      rasterReal.rebuild(surfaceImage, surfaceOffsetX, surfaceOffsetY, calibrated.offImageReflectance);
    }
    if (trackRobot) {
      viewCentreX = robot.odometry.pose.x;
      viewCentreY = robot.odometry.pose.y;
    }
    recordTelemetry();
  }

  /** @brief One fixed model step: clock, ghost, controller, physics. */
  void stepModel() {
    robot.advanceTime(simulationUpdateIntervalMs);
    updateLiveClock();
    controller.update(robot);
    robot.updatePhysics(simulationUpdateIntervalMs);
  }

  void updateRobotModel() {
    int now = millis();
    if (fastForward && simulatorMode == SANDBOX) {
      for (int k = 0; k < fastStepsPerFrame; k++) stepModel();
      lastSimulationUpdateMs = now;
      simulationAccumulatorMs = 0;
      return;
    }
    int elapsedMs = now - lastSimulationUpdateMs;
    lastSimulationUpdateMs = now;
    elapsedMs = min(elapsedMs, simulationUpdateIntervalMs * maximumCatchUpSteps);
    simulationAccumulatorMs += elapsedMs;
    int steps = 0;
    while (simulationAccumulatorMs >= simulationUpdateIntervalMs && steps < maximumCatchUpSteps) {
      stepModel();
      simulationAccumulatorMs -= simulationUpdateIntervalMs;
      steps++;
    }
  }

  void updateLiveClock() {
    if (simulatorMode != LIVE_COMPARISON || !liveRunning) return;
    long targetTcpTime = liveStartTcpMs + (robot.getMillis() - liveStartSimulationMs);
    while (tcpQueue.size() > 0 && tcpQueue.get(0).timestampMs <= targetTcpTime) {
      ghost = tcpQueue.remove(0);
    }
  }

  // ======================================================================
  // Twin scans
  // ======================================================================

  /** @brief Puts the twin at the start pose and starts a scan. */
  void beginSimRun() {
    controller.reset();
    controller.scan_log.clear();
    robot.setModel(activeModel);
    simRunSeed = (long) random(1000000);
    randomSeed(simRunSeed);
    // Placed by hand on the start cross: a little off, as in reality.
    robot.resetForRun(randomGaussian() * activeModel.startPositionSdMm,
      randomGaussian() * activeModel.startPositionSdMm,
      radians(randomGaussian() * activeModel.startHeadingSdDeg), 0, 0);
    rasterSim.setArea(controller.config, activeModel);
    truth = truthImage(rasterSim, surfaceImage, surfaceOffsetX, surfaceOffsetY, activeModel.offImageReflectance);
    simLogged = 0;
    simTrail.clear();
    runCounter++;
    runStamp = timestamp() + "_" + nf(runCounter, 3);
    controller.setSignal(1);
    simRunPending = true;
    simResult = "Twin: scanning...";
  }

  void stopSimRun() {
    controller.requestStop();
    repeatsRemaining = 0;
  }

  /** @brief Feeds new log entries to the twin's rebuild and trail. */
  void collectSimSamples() {
    ArrayList<ScanSample_c> log = controller.scan_log;
    while (simLogged < log.size()) {
      ScanSample_c s = log.get(simLogged++);
      if (s.state == controller.SCANNING_ROW || s.state == controller.OVERRUN) {
        rasterSim.addReading(s.x, s.y, s.theta, s.dn, activeModel);
      }
      simTrail.add(new PVector(s.true_x, s.true_y));
    }
  }

  void checkSimRunEnd() {
    if (!simRunPending || controller.start_requested) return;
    int m = controller.getMode();
    if (m == controller.FINISHED || m == controller.STOPPED) {
      simRunPending = false;
      onSimRunFinished();
    }
  }

  void onSimRunFinished() {
    collectSimSamples();
    rasterSim.dirty = true;
    rasterSim.rebuild(surfaceImage, surfaceOffsetX, surfaceOffsetY, activeModel.offImageReflectance);

    // Pose error: where the robot thought it was vs where it really was.
    ArrayList<ScanSample_c> log = controller.scan_log;
    float sumErr = 0, maxErr = 0;
    int n = 0;
    for (ScanSample_c s : log) {
      if (s.state != controller.SCANNING_ROW && s.state != controller.OVERRUN) continue;
      float e = dist(s.x, s.y, s.true_x, s.true_y);
      sumErr += e;
      maxErr = max(maxErr, e);
      n++;
    }
    float meanErr = n > 0 ? sumErr / n : 0;
    float headingErrDeg = 0, durationS = 0;
    if (log.size() > 0) {
      ScanSample_c last = log.get(log.size() - 1);
      headingErrDeg = degrees(atan2(sin(last.true_theta - last.theta), cos(last.true_theta - last.theta)));
      durationS = (last.t_ms - log.get(0).t_ms) / 1000.0;
    }

    String modelTag = activeModel == nominal ? "nominal" : "calibrated";
    String csv = "runs/" + runStamp + "_twin_" + modelTag + ".csv";
    writeScanCsv(sketchPath(csv), log, true, "Digital Twin run " + runStamp + "; model: "
      + activeModel.describe() + "; scan: " + controller.config.describe() + "; seed " + simRunSeed
      + "; " + controller.getMessage());
    rasterSim.image.save(sketchPath("runs/" + runStamp + "_twin_" + modelTag + ".png"));

    simResult = "Twin: " + controller.getMessage() + ". Coverage " + pct(rasterSim.coverage)
      + ", error vs sheet " + pct(rasterSim.maeVsTruth) + ", pose error mean "
      + nf(meanErr, 0, 1) + " / max " + nf(maxErr, 0, 1) + " mm";
    appendSummary(sketchPath("runs/summary.csv"), summaryHeader(),
      summaryLine("twin", modelTag, durationS, log.size(), rasterSim, -1, meanErr, maxErr, headingErrDeg, csv));
    savedMessage = "Saved " + csv;
    println(simResult);

    if (simulatorMode == LIVE_COMPARISON) {
      simDoneForLive = true;
      compareLiveRun();
      return;
    }
    if (realFromFile && rasterReal.count() > 0) compareLoadedScan();
    if (repeatsRemaining > 0) {
      repeatsRemaining--;
      beginSimRun();
    }
  }

  // ======================================================================
  // A saved robot scan (L): the dashboard's CSV, or the twin's own *_robot.csv
  // ======================================================================

  void chooseRobotCsv() {
    if (simulatorMode != SANDBOX) {
      savedMessage = "Loading a saved scan works in sandbox mode (press M first).";
      return;
    }
    selectInput("Choose a robot scan CSV (the dashboard's Download CSV)", "robotCsvSelected");
  }

  /**
   * @brief Reads a scan CSV, sets the twin's scan settings to match it, and
   * shows it as the real robot's rebuild. Accepts the dashboard format
   * (seq,t_ms,x_mm,y_mm,theta_rad,dn1..dn5,state,row) with or without a
   * header, and the template's 14-column telemetry log.
   */
  void loadRobotCsv(File file) {
    String[] lines = loadStrings(file.getAbsolutePath());
    if (lines == null) {
      savedMessage = "Could not read " + file.getName();
      return;
    }
    int it = -1, ix = -1, iy = -1, ith = -1, ist = -1, irow = -1;
    int[] idn = {-1, -1, -1, -1, -1};
    boolean first = true;
    ArrayList<ScanSample_c> loaded = new ArrayList<ScanSample_c>();
    for (String raw : lines) {
      String l = trim(raw);
      if (l.length() == 0 || l.startsWith("#")) continue;
      String[] f = split(l, ',');
      if (first) {
        first = false;
        if (l.matches(".*[a-df-zA-DF-Z_].*")) {           // a header row
          for (int k = 0; k < f.length; k++) {
            String h = trim(f[k]).toLowerCase();
            if (h.equals("t_ms") || h.equals("timestamp_ms") || h.equals("time_ms") || h.equals("t") || h.equals("ms")) it = k;
            if (h.equals("x_mm") || h.equals("x")) ix = k;
            if (h.equals("y_mm") || h.equals("y")) iy = k;
            if (h.equals("theta_rad") || h.equals("theta") || h.equals("th")) ith = k;
            for (int d = 0; d < 5; d++) if (h.equals("dn" + (d + 1))) idn[d] = k;
            if (h.equals("state")) ist = k;
            if (h.equals("row")) irow = k;
          }
          continue;
        } else if (f.length == 14) {                         // template telemetry
          it = 0; ix = 1; iy = 2; ith = 3;
          for (int d = 0; d < 5; d++) idn[d] = 8 + d;
        } else if (f.length >= 12) {                         // dashboard, no header
          it = 1; ix = 2; iy = 3; ith = 4; ist = 10; irow = 11;
          for (int d = 0; d < 5; d++) idn[d] = 5 + d;
        }
      }
      if (it < 0 || ix < 0 || iy < 0 || ith < 0 || idn[4] < 0) {
        savedMessage = file.getName() + ": needs t_ms, x, y, theta and dn1..dn5 columns.";
        return;
      }
      int need = max(max(max(it, ix), max(iy, ith)), max(idn[4], max(ist, irow)));
      if (f.length <= need) continue;
      ScanSample_c s = new ScanSample_c();
      float t = parseFloat(f[it]);
      s.x = parseFloat(f[ix]);
      s.y = parseFloat(f[iy]);
      s.theta = parseFloat(f[ith]);
      if (Float.isNaN(t) || Float.isNaN(s.x) || Float.isNaN(s.y) || Float.isNaN(s.theta)) continue;
      s.t_ms = (long) t;
      boolean ok = true;
      for (int d = 0; d < 5; d++) {
        s.dn[d] = parseFloat(f[idn[d]]);
        if (Float.isNaN(s.dn[d])) ok = false;
      }
      if (!ok) continue;
      s.state = ist >= 0 ? round(parseFloat(f[ist])) : -1;
      s.row = irow >= 0 ? round(parseFloat(f[irow])) : 0;
      s.true_x = Float.NaN;
      s.true_y = Float.NaN;
      s.true_theta = Float.NaN;
      loaded.add(s);
    }
    if (loaded.size() < 20) {
      savedMessage = file.getName() + ": not enough readable rows.";
      return;
    }

    // Work out the scan settings from the samples (rows, row length, pitch,
    // side, path), so the twin can scan exactly the same way.
    String settingsNote = "Scan settings kept (the file has no state/row columns).";
    boolean changed = false;
    if (ist >= 0 && irow >= 0) {
      RasterConfig_c c = controller.config.copy();
      int maxRow = 0;
      float maxX = -1e9;
      boolean uturn = false;
      float[] sumY = new float[256];
      int[] cnt = new int[256];
      for (ScanSample_c s : loaded) {
        if (s.state == controller.TURNING_OUT || s.state == controller.SHIFTING
          || s.state == controller.TURNING_IN || s.state == controller.OVERRUN) uturn = true;
        if (s.state != controller.SCANNING_ROW || s.row < 0 || s.row > 255) continue;
        maxRow = max(maxRow, s.row);
        if (s.row % 2 == 0) maxX = max(maxX, s.x);
        sumY[s.row] += s.y;
        cnt[s.row]++;
      }
      FloatList gaps = new FloatList();
      int prevRow = -1;
      float prevY = 0;
      for (int r = 0; r <= maxRow; r++) {
        if (cnt[r] == 0) continue;
        float m = sumY[r] / cnt[r];
        if (prevRow >= 0) gaps.append((m - prevY) / (r - prevRow));
        prevRow = r;
        prevY = m;
      }
      c.rows = maxRow + 1;
      c.pattern = uturn ? PATTERN_UTURN : PATTERN_BACK_AND_FORTH;
      if (maxX > 20) c.row_length_mm = round(maxX);
      if (gaps.size() > 0) {
        gaps.sort();
        float pitch = gaps.get(gaps.size() / 2);
        c.side = pitch < 0 ? -1 : 1;
        c.row_pitch_mm = max(2, round(abs(pitch) * 2) / 2.0);
      }
      changed = !c.describe().equals(controller.config.describe()) || c.side != controller.config.side;
      if (!controller.isRunning() && controller.setConfig(c)) {
        settingsNote = "Twin set to the scan's settings: " + controller.config.describe() + ".";
      } else {
        settingsNote = "Stop the twin (X) and load again to copy the scan's settings.";
      }
    }

    // No state column: label the scan rows like the live comparison does.
    RasterConfig_c c = controller.config;
    for (ScanSample_c s : loaded) {
      if (s.state >= 0) continue;
      boolean alongRow = abs(sin(s.theta)) < 0.35;
      s.state = (alongRow && s.x >= 0 && s.x <= c.row_length_mm) ? controller.SCANNING_ROW : controller.MARGIN;
      s.row = constrain(round(s.y / (c.side * c.row_pitch_mm)), 0, c.rows - 1);
    }

    realFromFile = true;
    realSource = file.getName();
    rasterReal.setArea(controller.config, calibrated);
    realSamples.clear();
    realTrail.clear();
    for (ScanSample_c s : loaded) {
      realSamples.add(s);
      realTrail.add(new PVector(s.x, s.y));
      if (s.state == controller.SCANNING_ROW || s.state == controller.OVERRUN) {
        rasterReal.addReading(s.x, s.y, s.theta, s.dn, calibrated);
      }
    }
    rasterReal.dirty = true;
    rasterReal.rebuild(surfaceImage, surfaceOffsetX, surfaceOffsetY, calibrated.offImageReflectance);
    float durationS = (loaded.get(loaded.size() - 1).t_ms - loaded.get(0).t_ms) / 1000.0;
    realResult = "Robot (" + realSource + "): coverage " + pct(rasterReal.coverage)
      + ", error vs sheet " + pct(rasterReal.maeVsTruth);
    appendSummary(sketchPath("runs/summary.csv"), summaryHeader(),
      summaryLine("robot", "-", durationS, loaded.size(), rasterReal, -1, -1, -1, 0, realSource));
    configStatus = settingsNote;
    simVsReal = -1;
    if (!changed && !simRunPending && rasterSim.count() > 0) {
      compareLoadedScan();
    } else {
      simResult = "Loaded " + realSource + " (" + loaded.size() + " samples). Press S to scan the twin the same way;"
        + " the twin vs robot score appears when it finishes.";
    }
    println(realResult);
  }

  /** @brief Twin's last scan vs the loaded robot scan. */
  void compareLoadedScan() {
    simVsReal = compareRebuilds(rasterSim, rasterReal);
    String modelTag = activeModel == nominal ? "nominal" : "calibrated";
    appendSummary(sketchPath("runs/summary.csv"), summaryHeader(),
      summaryLine("twin_vs_robot", modelTag, 0, 0, rasterSim, simVsReal, -1, -1, 0, realSource));
    savedMessage = "Twin vs robot (" + realSource + "): " + pct(simVsReal) + " different. Saved in runs/summary.csv";
    println(savedMessage);
  }

  // ======================================================================
  // Real robot scans (live comparison)
  // ======================================================================

  /** @brief Starts the twin together with the real robot. */
  void startLiveRun(TCPTelemetry_c sample) {
    beginSimRun();
    realFromFile = false;
    realSource = "live";
    rasterReal.setArea(controller.config, calibrated);
    realSamples.clear();
    realTrail.clear();
    simDoneForLive = false;
    realDoneForLive = false;
    simVsReal = -1;
    realResult = "Robot: scanning...";
    liveStartTcpMs = sample.timestampMs;
    liveStartSimulationMs = robot.getMillis();
    ghost = sample.copy();
    liveRunning = true;
    tcpQueue.clear();
  }

  /** @brief Turns one TCP telemetry line into a scan sample of the real robot. */
  void recordRealSample(TCPTelemetry_c t) {
    RasterConfig_c c = controller.config;
    if (realSamples.size() > 0) {
      ScanSample_c prev = realSamples.get(realSamples.size() - 1);
      if (dist(prev.x, prev.y, t.x, t.y) < 0.2) return;  // standing still (start delay, pauses)
    }
    ScanSample_c s = new ScanSample_c();
    s.t_ms = t.timestampMs;
    s.x = t.x;
    s.y = t.y;
    s.theta = t.theta;
    for (int i = 0; i < 5; i++) s.dn[i] = t.sensorReadings[i];
    boolean alongRow = abs(sin(t.theta)) < 0.35;
    s.state = (alongRow && t.x >= 0 && t.x <= c.row_length_mm) ? controller.SCANNING_ROW : controller.MARGIN;
    s.row = constrain(round(t.y / (c.side * c.row_pitch_mm)), 0, c.rows - 1);
    s.true_x = Float.NaN;
    s.true_y = Float.NaN;
    s.true_theta = Float.NaN;
    realSamples.add(s);
    realTrail.add(new PVector(t.x, t.y));
    if (s.state == controller.SCANNING_ROW) rasterReal.addReading(s.x, s.y, s.theta, s.dn, calibrated);
  }

  void onRealRunFinished() {
    liveRunning = false;
    rasterReal.dirty = true;
    rasterReal.rebuild(surfaceImage, surfaceOffsetX, surfaceOffsetY, calibrated.offImageReflectance);
    float durationS = realSamples.size() > 1
      ? (realSamples.get(realSamples.size() - 1).t_ms - realSamples.get(0).t_ms) / 1000.0 : 0;
    String csv = "runs/" + runStamp + "_robot.csv";
    writeScanCsv(sketchPath(csv), realSamples, false, "Real robot run " + runStamp
      + " from TCP telemetry (20 Hz); scan: " + controller.config.describe());
    rasterReal.image.save(sketchPath("runs/" + runStamp + "_robot.png"));
    realResult = "Robot: coverage " + pct(rasterReal.coverage) + ", error vs sheet " + pct(rasterReal.maeVsTruth);
    appendSummary(sketchPath("runs/summary.csv"), summaryHeader(),
      summaryLine("robot", "-", durationS, realSamples.size(), rasterReal, -1, -1, -1, 0, csv));
    savedMessage = "Saved " + csv;
    println(realResult);
    realDoneForLive = true;
    compareLiveRun();
  }

  /** @brief When both the twin and the robot have finished, compares the rebuilds. */
  void compareLiveRun() {
    if (!simDoneForLive || !realDoneForLive) return;
    simVsReal = compareRebuilds(rasterSim, rasterReal);
    String modelTag = activeModel == nominal ? "nominal" : "calibrated";
    appendSummary(sketchPath("runs/summary.csv"), summaryHeader(),
      summaryLine("twin_vs_robot", modelTag, 0, 0, rasterSim, simVsReal, -1, -1, 0, ""));
    println("Twin vs robot image difference: " + pct(simVsReal));
  }

  String summaryHeader() {
    return "run,source,model,pattern,rows,row_len_mm,pitch_mm,scan_pwm,seed,duration_s,samples,"
      + "coverage_pct,error_vs_sheet_pct,twin_vs_robot_pct,pose_err_mean_mm,pose_err_max_mm,"
      + "final_heading_err_deg,file";
  }

  String summaryLine(String source, String model, float durationS, int samples, RasterImage_c r,
    float twinVsRobot, float meanErr, float maxErr, float headingErr, String file) {
    RasterConfig_c c = controller.config;
    return runStamp + "," + source + "," + model + "," + (c.pattern == PATTERN_UTURN ? "uturn" : "back_and_forth")
      + "," + c.rows + "," + fmtUS(c.row_length_mm, 1) + "," + fmtUS(c.row_pitch_mm, 2) + "," + fmtUS(c.scan_pwm, 1)
      + "," + (source.equals("robot") ? "" : "" + simRunSeed) + "," + fmtUS(durationS, 2) + "," + samples
      + "," + (source.equals("twin_vs_robot") ? "" : fmtUS(100 * r.coverage, 1))
      + "," + (source.equals("twin_vs_robot") || r.maeVsTruth < 0 ? "" : fmtUS(100 * r.maeVsTruth, 2))
      + "," + (twinVsRobot < 0 ? "" : fmtUS(100 * twinVsRobot, 2))
      + "," + (meanErr < 0 ? "" : fmtUS(meanErr, 2)) + "," + (maxErr < 0 ? "" : fmtUS(maxErr, 2))
      + "," + (source.equals("twin") ? fmtUS(headingErr, 2) : "") + "," + file;
  }

  // ======================================================================
  // Robot settings from the dashboard
  // ======================================================================

  void requestRobotConfig() {
    if (configFetchInProgress) return;
    lastConfigFetchMs = millis();
    configFetchInProgress = true;
    new Thread(new Runnable() {
      public void run() {
        try {
          JSONObject j = loadJSONObject(robotConfigUrl);
          if (j == null) throw new RuntimeException("no reply");
          fetchedConfig = j;
        }
        catch (Exception e) {
          configFetchError = "Scan settings: couldn't read them from the robot (on its WiFi?). Using the twin's.";
        }
        configFetchInProgress = false;
      }
    }).start();
  }

  void applyFetchedConfig() {
    if (configFetchError != null) {
      configStatus = configFetchError;
      configFetchError = null;
    }
    if (fetchedConfig == null || controller.isRunning()) return;
    JSONObject j = fetchedConfig;
    fetchedConfig = null;
    JSONObject cfg = j.hasKey("cfg") ? j.getJSONObject("cfg") : j;
    if (controller.applyRobotConfig(cfg)) {
      // Takes effect at the next scan (the rebuilds on screen are kept).
      configStatus = "Scan settings copied from the robot: " + controller.config.describe();
    } else {
      configStatus = "Scan settings: the robot's were out of range. Using the twin's.";
    }
  }

  // ======================================================================
  // Drawing
  // ======================================================================

  void draw() {
    if (editing) {
      editor.draw();
      return;
    }
    background(180);
    updateSurfaceViewportCache();

    pushMatrix();
    translate(worldViewCentreX(), height / 2.0);
    scale(zoom, -zoom);
    translate(-viewCentreX, -viewCentreY);
    imageMode(CENTER);
    drawSurfaceViewportCache();
    if (overlay == 1 && rasterReal.count() > 0) drawOverlay(rasterReal);
    if (overlay == 2 && rasterSim.count() > 0) drawOverlay(rasterSim);
    drawScanArea();
    drawTrail(realTrail, color(235, 125, 35, 170));
    drawTrail(simTrail, color(40, 100, 220, 150));
    robot.draw();
    if (simulatorMode == LIVE_COMPARISON && ghost.timestampMs >= 0) drawGhost();
    popMatrix();

    drawStatus();
    if (showTelemetry) drawTelemetryPanel();
    else if (showRaster) drawRasterPanel();
  }

  /** @return Screen x of the world view's centre (left of any side panel). */
  float worldViewCentreX() {
    float panel = (showTelemetry || showRaster) ? telemetryPanelWidth() : 0;
    return (width - panel) / 2.0;
  }

  /** @brief Lays a rebuilt image over the sheet, at the place it was scanned. */
  void drawOverlay(RasterImage_c r) {
    pushStyle();
    pushMatrix();
    translate(r.x0, r.y0 + r.h);
    scale(1, -1);
    imageMode(CORNER);
    tint(255, 215);
    image(r.image, 0, 0, r.w, r.h);
    popMatrix();
    popStyle();
  }

  void drawScanArea() {
    pushStyle();
    noFill();
    stroke(40, 100, 220, 120);
    strokeWeight(1.0 / zoom);
    rectMode(CORNERS);
    rect(rasterSim.tx0, rasterSim.ty0, rasterSim.tx1, rasterSim.ty1);
    popStyle();
  }

  void drawTrail(ArrayList<PVector> trail, int c) {
    if (trail.size() < 2) return;
    pushStyle();
    noFill();
    stroke(c);
    strokeWeight(1.2 / zoom);
    beginShape();
    for (PVector p : trail) vertex(p.x, p.y);
    endShape();
    popStyle();
  }

  void drawRasterPanel() {
    int margin = uiMargin();
    int panelWidth = telemetryPanelWidth();
    int panelX = width - panelWidth;
    float titleHeight = max(28, 24 * uiScale());
    float textH = max(16, 15 * uiScale());
    float imgW = panelWidth - 2 * margin;
    float imgH = min((height - titleHeight - margin * 4 - textH * 6) / 3.0,
      imgW * rasterSim.h / (float) max(1, rasterSim.w));
    imgH = max(20, imgH);

    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(248, 248, 248, 240);
    rect(panelX, 0, panelWidth, height);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Image rebuilds (R hides, G for graphs)", panelX + margin, titleHeight - margin / 2.0);

    float y = titleHeight + margin;
    fill(60);
    text("Printed sheet (truth)", panelX + margin, y + textH * 0.8);
    y += textH;
    if (truth != null) {
      float s = min(imgW / truth.width, imgH / truth.height);
      imageMode(CORNER);
      image(truth, panelX + margin + (imgW - truth.width * s) / 2, y, truth.width * s, truth.height * s);
    }
    y += imgH + margin;

    fill(40, 100, 220);
    text("Twin - " + activeModel.name, panelX + margin, y + textH * 0.8);
    y += textH;
    rasterSim.draw(panelX + margin, y, imgW, imgH);
    y += imgH + 2;
    fill(60);
    text(simResultShort(), panelX + margin, y + textH * 0.8);
    y += textH + margin;

    fill(200, 100, 20);
    text(realFromFile ? "Real robot - " + realSource : "Real robot (M live, L load a scan CSV)", panelX + margin, y + textH * 0.8);
    y += textH;
    rasterReal.draw(panelX + margin, y, imgW, imgH);
    y += imgH + 2;
    fill(60);
    String realLine = rasterReal.count() == 0 ? "No robot scan yet. Press L to load one."
      : "Coverage " + pct(rasterReal.coverage) + ", error vs sheet " + pct(rasterReal.maeVsTruth);
    if (simVsReal >= 0) realLine += ", twin vs robot " + pct(simVsReal);
    text(realLine, panelX + margin, y + textH * 0.8);
    popStyle();
  }

  String simResultShort() {
    if (rasterSim.count() == 0) return "No samples yet.";
    return "Coverage " + pct(rasterSim.coverage) + ", error vs sheet " + pct(rasterSim.maeVsTruth)
      + ", pose error " + nf(dist(controller.odometry.pose.x, controller.odometry.pose.y,
      robot.odometry.pose.x, robot.odometry.pose.y), 0, 1) + " mm now";
  }

  void updateSurfaceViewportCache() {
    float halfViewWidth = width / (2.0 * zoom);
    float halfViewHeight = height / (2.0 * zoom);
    int visibleX0 = floor(surfaceImage.width / 2.0 + viewCentreX - surfaceOffsetX - halfViewWidth);
    int visibleX1 = ceil(surfaceImage.width / 2.0 + viewCentreX - surfaceOffsetX + halfViewWidth);
    int visibleY0 = floor(surfaceImage.height / 2.0 - viewCentreY + surfaceOffsetY - halfViewHeight);
    int visibleY1 = ceil(surfaceImage.height / 2.0 - viewCentreY + surfaceOffsetY + halfViewHeight);

    visibleX0 = constrain(visibleX0, 0, surfaceImage.width);
    visibleX1 = constrain(visibleX1, 0, surfaceImage.width);
    visibleY0 = constrain(visibleY0, 0, surfaceImage.height);
    visibleY1 = constrain(visibleY1, 0, surfaceImage.height);
    if (visibleX1 <= visibleX0 || visibleY1 <= visibleY0) {
      surfaceViewportCache = null;
      return;
    }
    boolean cacheContainsView = surfaceViewportCache != null
      && visibleX0 >= surfaceCacheX
      && visibleY0 >= surfaceCacheY
      && visibleX1 <= surfaceCacheX + surfaceCacheWidth
      && visibleY1 <= surfaceCacheY + surfaceCacheHeight;
    if (cacheContainsView) return;

    int cacheMargin = ceil(constrain(max(halfViewWidth, halfViewHeight) * 0.15, 40, 250));
    surfaceCacheX = max(0, visibleX0 - cacheMargin);
    surfaceCacheY = max(0, visibleY0 - cacheMargin);
    int cacheRight = min(surfaceImage.width, visibleX1 + cacheMargin);
    int cacheBottom = min(surfaceImage.height, visibleY1 + cacheMargin);
    surfaceCacheWidth = cacheRight - surfaceCacheX;
    surfaceCacheHeight = cacheBottom - surfaceCacheY;
    surfaceViewportCache = surfaceImage.get(surfaceCacheX, surfaceCacheY, surfaceCacheWidth, surfaceCacheHeight);
  }

  void drawSurfaceViewportCache() {
    if (surfaceViewportCache == null) return;
    float worldX = surfaceOffsetX + surfaceCacheX + surfaceCacheWidth / 2.0 - surfaceImage.width / 2.0;
    float worldY = surfaceOffsetY + surfaceImage.height / 2.0 - (surfaceCacheY + surfaceCacheHeight / 2.0);
    // The world is drawn upside down (scale(zoom, -zoom)); flip the image back.
    pushMatrix();
    translate(worldX, worldY);
    scale(1, -1);
    image(surfaceViewportCache, 0, 0);
    popMatrix();
  }

  void drawGhost() {
    float alpha = millis() - ghost.receivedAtMs > 100 ? 128 : 255;
    pushStyle();
    pushMatrix();
    translate(ghost.x, ghost.y);
    rotate(ghost.theta);
    fill(235, 125, 35, alpha * 0.5);
    stroke(180, 70, 15, alpha);
    strokeWeight(1.5 / zoom);
    ellipse(0, 0, 48.5, 48.5);
    line(0, 0, 36, 0);
    popMatrix();
    popStyle();
  }

  // ======================================================================
  // Keys
  // ======================================================================

  void handleKey(char pressedKey, int pressedKeyCode) {
    if (editing) {
      if (pressedKey == ENTER || pressedKey == RETURN) finishEdit();
      else editor.handleKey(pressedKey);
      return;
    }
    char k = Character.toLowerCase(pressedKey);
    if (k == 'g') { showTelemetry = !showTelemetry; if (showTelemetry) showRaster = false; }
    if (k == 'r') { showRaster = !showRaster; if (showRaster) showTelemetry = false; }
    if (k == 't') trackRobot = !trackRobot;
    if (k == 'm' && !liveRunning) toggleMode();
    if (k == 's' && simulatorMode == SANDBOX) { repeatsRemaining = 0; beginSimRun(); }
    if (k == 'x') stopSimRun();
    if (k == 'f') fastForward = !fastForward;
    if (k == 'n' && simulatorMode == SANDBOX) {
      repeatsRemaining = 9;   // this run + 9 more = 10 repeats
      fastForward = true;
      beginSimRun();
    }
    if (k == 'p' && !controller.isRunning() && !liveRunning) {
      activeModel = (activeModel == calibrated) ? nominal : calibrated;
      robot.setModel(activeModel);
      rasterSim.setArea(controller.config, activeModel);
      truth = truthImage(rasterSim, surfaceImage, surfaceOffsetX, surfaceOffsetY, activeModel.offImageReflectance);
      simResult = "Model: " + activeModel.name + ". Press S to scan.";
    }
    if (k == 'c') requestRobotConfig();
    if (k == 'l') chooseRobotCsv();
    if (k == 'i' && !liveRunning) chooseSheetImage();
    if (k == 'e' && !liveRunning && editor.hasPicture()) editing = true;
    if (k == 'o') {
      overlay = (overlay + 1) % 3;
      String[] what = {"Overlay off: the printed sheet", "Overlay: robot's rebuild on the sheet", "Overlay: twin's rebuild on the sheet"};
      savedMessage = what[overlay] + " (O to change)";
    }
    if (pressedKey == '+' || pressedKey == '=') zoom = min(maximumZoom, zoom * 1.2);
    if (pressedKey == '-') zoom = max(minimumZoom, zoom / 1.2);
    if (pressedKey == CODED) {
      float panStepMm = 50.0 / zoom;
      if (pressedKeyCode == LEFT) panBy(-panStepMm, 0);
      if (pressedKeyCode == RIGHT) panBy(panStepMm, 0);
      if (pressedKeyCode == UP) panBy(0, panStepMm);
      if (pressedKeyCode == DOWN) panBy(0, -panStepMm);
    }
  }

  void toggleMode() {
    simulatorMode = simulatorMode == SANDBOX ? LIVE_COMPARISON : SANDBOX;
    resetLiveSession();
    repeatsRemaining = 0;
    fastForward = false;
    if (simulatorMode == SANDBOX) {
      beginSimRun();
    } else {
      controller.reset();
      simRunPending = false;
      robot.resetForRun(0, 0, 0, 0, 0);
      simResult = "Waiting for the robot to start a scan.";
      requestRobotConfig();
    }
  }

  void panBy(float deltaX, float deltaY) {
    trackRobot = false;
    viewCentreX += deltaX;
    viewCentreY += deltaY;
  }

  // ======================================================================
  // Status text and graphs
  // ======================================================================

  void drawStatus() {
    int margin = uiMargin();
    int contentWidth = statusWidth();
    float lineHeight = max(18, 16 * uiScale());
    RobotPose_c t = robot.odometry.pose;
    RobotPose_c e = controller.odometry.pose;
    String[] lines = {
      "Twin " + controller.getModeName() + "  row " + (controller.getRow() + 1) + "/" + controller.config.rows
        + "   true pose " + nf(t.x, 0, 1) + ", " + nf(t.y, 0, 1) + ", " + nf(degrees(t.theta), 0, 1) + " deg"
        + "   robot's estimate " + nf(e.x, 0, 1) + ", " + nf(e.y, 0, 1),
      "Model: " + activeModel.name + " (P)   " + (fastForward ? "FAST (F)" : "real time (F = fast)")
        + (repeatsRemaining > 0 ? "   repeats left: " + repeatsRemaining : ""),
      configStatus,
      simResult,
      tcpStatus + "   " + liveStatus(),
      "S scan  X stop  N 10 repeats  P model  I load picture  E edit picture  L load robot scan  O overlay  C robot settings  M mode  R rebuilds  G graphs  T track  +/- zoom  arrows pan",
      savedMessage
    };
    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(255, 255, 255, 215);
    rect(margin - 6, margin - 6, contentWidth + 12, lineHeight * lines.length + 12, 5);
    fill(30);
    textSize(max(11, 12 * uiScale()));
    for (int i = 0; i < lines.length; i++) {
      text(lines[i], margin, margin + lineHeight * i, contentWidth, lineHeight);
    }
    popStyle();
  }

  String liveStatus() {
    if (simulatorMode == SANDBOX) return "Mode: Sandbox (M = live comparison)";
    if (telemetryClient == null) return "Mode: Live comparison - offline (join the robot's WiFi)";
    if (!liveArmed) return "Mode: Live comparison - waiting for the robot to be idle";
    if (!liveRunning) return "Mode: Live comparison - armed: start a scan on the dashboard";
    return "Mode: Live comparison - running; orange = robot's own odometry";
  }

  void recordTelemetry() {
    int now = millis();
    if (now - lastGraphSampleMs < graphSamplePeriodMs) return;
    lastGraphSampleMs = now;
    float[] simulatedSensors = robot.sensorReadings();
    if (tcpTelemetry.isRecent(now)) {
      motorGraph.addSample(now, new float[] {robot.motors.left.pwm, robot.motors.right.pwm,
        tcpTelemetry.pwmLeft, tcpTelemetry.pwmRight});
      sensorGraph.addSample(now, combine(simulatedSensors, tcpTelemetry.sensorReadings));
      encoderGraph.addSample(now, new float[] {robot.odometry.encoders.left,
        robot.odometry.encoders.right, tcpTelemetry.encoderLeft, tcpTelemetry.encoderRight});
    } else {
      motorGraph.addSample(now, new float[] {robot.motors.left.pwm, robot.motors.right.pwm});
      sensorGraph.addSample(now, simulatedSensors);
      encoderGraph.addSample(now, new float[] {robot.odometry.encoders.left, robot.odometry.encoders.right});
    }
  }

  float[] combine(float[] first, float[] second) {
    float[] combined = new float[first.length + second.length];
    for (int i = 0; i < first.length; i++) combined[i] = first[i];
    for (int i = 0; i < second.length; i++) combined[first.length + i] = second[i];
    return combined;
  }

  void drawTelemetryPanel() {
    int margin = uiMargin();
    int panelWidth = telemetryPanelWidth();
    int panelX = width - panelWidth;
    int graphWidth = panelWidth - 2 * margin;
    float titleHeight = max(28, 24 * uiScale());
    float graphGap = margin;
    float graphHeight = (height - titleHeight - margin * 4 - graphGap * 2) / 3.0;
    graphHeight = max(1, graphHeight);
    float graphY = titleHeight + margin;
    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(248, 248, 248, 235);
    rect(panelX, 0, panelWidth, height);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Telemetry (press G to hide)", panelX + margin, titleHeight - margin / 2.0);
    motorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    sensorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    encoderGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    popStyle();
  }

  float uiScale() {
    return constrain(min(width / 1000.0, height / 800.0), 0.85, 1.5);
  }

  int uiMargin() {
    return max(10, round(12 * uiScale()));
  }

  int telemetryPanelWidth() {
    int availableWidth = max(180, width - 2 * uiMargin());
    int preferredWidth = round(width * 0.34);
    return min(max(280, preferredWidth), availableWidth);
  }

  int statusWidth() {
    int availableRight = (showTelemetry || showRaster) ? width - telemetryPanelWidth() : width;
    return max(1, availableRight - 2 * uiMargin());
  }

  int sensorColour(int index) {
    int[] colours = {color(160, 60, 60), color(210, 120, 50),
      color(80, 150, 80), color(60, 120, 190), color(140, 70, 170)};
    return colours[index];
  }

  int tcpSensorColour(int index) {
    int[] colours = {color(225, 110, 110), color(245, 180, 70),
      color(105, 190, 105), color(80, 160, 230), color(185, 100, 210)};
    return colours[index];
  }

  String pct(float fraction) {
    return fraction < 0 ? "n/a" : nf(100 * fraction, 0, 1) + "%";
  }

  String timestamp() {
    return nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "-" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  }

  /** @brief Puts the scan area in view. */
  void frameView() {
    viewCentreX = controller.config.row_length_mm / 2.0 + 40;
    viewCentreY = controller.config.side * (controller.config.rows - 1) * controller.config.row_pitch_mm / 2.0;
    float spanX = controller.config.row_length_mm + 220;
    float spanY = (controller.config.rows - 1) * controller.config.row_pitch_mm + 200;
    zoom = constrain(min((width - telemetryPanelWidth()) / spanX, height / spanY), minimumZoom, maximumZoom);
  }

  // ======================================================================
  // Your own picture as the sheet (I)
  // ======================================================================

  void chooseSheetImage() {
    selectInput("Choose the picture to scan (PNG or JPG), or a make_sheet .json", "sheetImageSelected");
  }

  /** @brief Loads a picture (or a ready-made sheet) and makes it the twin's sheet. */
  void loadSheetFile(File f) {
    String n = f.getName();
    String lower = n.toLowerCase();
    String base = n.replaceAll("\\.[^.]*$", "").replaceAll("[^A-Za-z0-9_-]", "_");
    if (lower.endsWith(".pdf")) {
      savedMessage = "PDFs can't be loaded: open it in Preview, File > Export, PNG (150 pixels/inch), and load the PNG.";
      return;
    }
    // A sheet made by make_sheet.py: its .json next to its _surface.png.
    if (lower.endsWith(".json")) {
      JSONObject j = loadJSONObject(f.getAbsolutePath());
      File png = new File(f.getParentFile(), base + "_surface.png");
      PImage img = png.exists() ? loadImage(png.getAbsolutePath()) : null;
      if (j == null || img == null || !j.hasKey("offset_x_mm")) {
        savedMessage = n + ": expected a make_sheet.py .json with its _surface.png beside it.";
        return;
      }
      img.save(sketchPath("surface.png"));
      saveJSONObject(j, sketchPath("surface.json"));
      installSheet(img, j.getFloat("offset_x_mm"), j.getFloat("offset_y_mm"),
        j.hasKey("settings") ? j.getJSONObject("settings") : null, j.getString("sheet", base));
      return;
    }
    byte[] bytes = loadBytes(f.getAbsolutePath());
    PImage pic = loadImage(f.getAbsolutePath());
    if (pic == null || pic.width <= 0) {
      savedMessage = n + " could not be opened as a picture (PNG, JPG, GIF or TGA).";
      return;
    }
    // A *_template.png from make_sheet.py: a whole sheet with its placement inside.
    JSONObject place = pngTextJson(bytes, "rasterbot-template");
    if (place != null && place.hasKey("w_mm")) {
      float w = place.getFloat("w_mm"), h = place.getFloat("h_mm");
      PImage surf = createImage(round(w), round(h), RGB);
      surf.copy(pic, 0, 0, pic.width, pic.height, 0, 0, surf.width, surf.height);
      float ox = place.getFloat("x0_mm") + w / 2, oy = place.getFloat("y0_mm") + h / 2;
      JSONObject j = new JSONObject();
      j.setString("sheet", place.getString("sheet", base));
      j.setFloat("offset_x_mm", ox);
      j.setFloat("offset_y_mm", oy);
      surf.save(sketchPath("surface.png"));
      saveJSONObject(j, sketchPath("surface.json"));
      installSheet(surf, ox, oy, null, j.getString("sheet"));
      return;
    }
    // Any picture: rotate and crop it first, then lay it out like make_sheet.py.
    editor.open(pic, n, base, imagePxPerMm(bytes));
    editing = true;
    savedMessage = "Rotate and crop the picture, then press Enter.";
  }

  void finishEdit() {
    editing = false;
    makeSheetFromPicture(editor.result(), editor.fileName, editor.base, editor.pxPerMm);
  }

  void cancelEdit() {
    editing = false;
    savedMessage = "Picture not used (E opens it again).";
  }

  /** @brief Asks the printed width, lays the picture out, writes the PDF and switches to it. */
  void makeSheetFromPicture(PImage pic, String n, String base, float ppm) {
    float suggested = (!Float.isNaN(ppm) && ppm > 1.9 && ppm < 200) ? pic.width / ppm : 150;
    float w = askPrintedWidth(n, round(suggested * 10) / 10.0);
    if (Float.isNaN(w)) {
      savedMessage = "Cancelled (the width must be 10 to 400 mm).";
      return;
    }
    float h = w * pic.height / (float) pic.width;
    SheetLayout_c L = layoutSheet(base, w, h, controller.config.row_pitch_mm);
    if (L == null) {
      savedMessage = n + " at " + nf(w, 0, 0) + " x " + nf(h, 0, 0) + " mm doesn't fit on A3 with the robot's start: try it smaller.";
      return;
    }
    PImage surf = renderSheetSurface(pic, L);
    JSONObject j = L.toJson();
    surf.save(sketchPath("sheets/" + base + "_surface.png"));
    saveJSONObject(j, sketchPath("sheets/" + base + ".json"));
    String pdfNote;
    try {
      writeSheetPdf(pic, L, sketchPath("sheets/" + base + ".pdf"));
      pdfNote = "Print sheets/" + base + ".pdf at 100% on " + L.pageName.split(" ")[0] + ". ";
    }
    catch (Exception e) {
      pdfNote = "(The PDF could not be written: " + e.getMessage() + ") ";
    }
    surf.save(sketchPath("surface.png"));
    saveJSONObject(j, sketchPath("surface.json"));
    installSheet(surf, j.getFloat("offset_x_mm"), j.getFloat("offset_y_mm"), j.getJSONObject("settings"), base);
    savedMessage = pdfNote + "Dashboard: Rows " + L.rows + ", Row length " + round(L.rowLen)
      + " mm, Row pitch " + nf(L.pitch, 0, 1) + " mm.";
    println(savedMessage);
  }

  /** @brief Switches the twin to a new sheet and starts a fresh scan of it. */
  void installSheet(PImage img, float offX, float offY, JSONObject settings, String name) {
    controller.reset();
    simRunPending = false;
    repeatsRemaining = 0;
    img.loadPixels();
    surfaceImage = img;
    surfaceOffsetX = offX;
    surfaceOffsetY = offY;
    robot.setSurface(img, offX, offY);
    surfaceViewportCache = null;
    surfaceCacheX = -1;
    sheetName = name;
    if (settings != null) controller.applyRobotConfig(settings);
    configStatus = "Sheet \"" + name + "\": " + controller.config.describe() + " (C copies the robot's)";
    rasterReal.setArea(controller.config, calibrated);
    realSamples.clear();
    realTrail.clear();
    realFromFile = false;
    simVsReal = -1;
    frameView();
    if (simulatorMode == SANDBOX) beginSimRun();
  }

  /** @return surface.json from the sketch folder, or null if there is none. */
  JSONObject loadSheetInfo() {
    File f = new File(sketchPath("surface.json"));
    if (!f.exists()) return null;
    try {
      return loadJSONObject(f.getAbsolutePath());
    }
    catch (Exception e) {
      println("surface.json could not be read: " + e.getMessage());
      return null;
    }
  }

  /**
   * @brief Loads surface.png (the printed sheet at 1 px/mm), or makes a
   * plain white sheet if it is missing.
   */
  PImage loadSurfaceImage() {
    PImage image = loadImage("surface.png");
    if (image != null && image.width > 0 && image.height > 0) return image;
    println("surface.png not found: using a blank A3 sheet.");
    image = createImage(420, 297, RGB);
    image.loadPixels();
    for (int i = 0; i < image.pixels.length; i++) image.pixels[i] = color(255);
    image.updatePixels();
    return image;
  }

  // ======================================================================
  // TCP (template code, plus the live-comparison hooks)
  // ======================================================================

  void updateTcpConnection() {
    int now = millis();
    if (telemetryClient != null && !telemetryClient.active()) {
      telemetryClient.stop();
      telemetryClient = null;
      tcpStatus = "TCP: disconnected; retrying";
      resetLiveSession();
    }
    if (pendingTcpSocket != null) {
      Socket connectedSocket = pendingTcpSocket;
      pendingTcpSocket = null;
      tcpAttemptInProgress = false;
      try {
        telemetryClient = new Client(applet, connectedSocket);
        tcpStatus = "TCP: connected to " + tcpHost + ":" + tcpPort;
      }
      catch (Exception error) {
        telemetryClient = null;
        tcpStatus = "TCP: unavailable; retrying";
        closeSocket(connectedSocket);
      }
    }
    if (tcpAttemptFailed) {
      tcpAttemptFailed = false;
      tcpAttemptInProgress = false;
      tcpStatus = "TCP: unavailable; retrying";
    }
    if (telemetryClient == null && !tcpAttemptInProgress && now - lastTcpAttemptMs >= tcpRetryPeriodMs) {
      beginTcpConnectionAttempt();
    }
    int linesRead = 0;
    while (telemetryClient != null && telemetryClient.available() > 0 && linesRead < 20) {
      String line = telemetryClient.readStringUntil('\n');
      if (line == null) break;
      lastTcpLine = trim(line);
      if (tcpTelemetry.parse(lastTcpLine, now)) handleTcpSample(tcpTelemetry);
      linesRead++;
    }
  }

  /**
   * @brief Uses a robot telemetry record for graphing and live comparison.
   * @details The robot's signal column is 1 while it scans and 0 otherwise.
   * 0 -> 1 starts the twin's scan at the same moment; 1 -> 0 ends the robot's run.
   */
  void handleTcpSample(TCPTelemetry_c sample) {
    if (lastTcpTimestampMs >= 0 && sample.timestampMs < lastTcpTimestampMs) resetLiveSession();
    lastTcpTimestampMs = sample.timestampMs;
    if (simulatorMode != LIVE_COMPARISON) return;
    TCPTelemetry_c copy = sample.copy();

    if (liveRunning) {
      tcpQueue.add(copy);
      if (sample.signal == 1) recordRealSample(copy);
    }
    if (sample.signal == 0) {
      if (!liveArmed || millis() - lastConfigFetchMs > 10000) requestRobotConfig();
      armedPose = copy;
      liveArmed = true;
      if (liveRunning && previousTcpSignal == 1) onRealRunFinished();
    }
    if (liveArmed && !liveRunning && previousTcpSignal == 0 && sample.signal == 1) startLiveRun(sample);
    previousTcpSignal = sample.signal;
  }

  void resetLiveSession() {
    tcpQueue.clear();
    liveArmed = false;
    liveRunning = false;
    previousTcpSignal = 0;
    lastTcpTimestampMs = -1;
    ghost = new TCPTelemetry_c();
    if (simulatorMode == LIVE_COMPARISON) {
      controller.reset();
      simRunPending = false;
    }
  }

  void beginTcpConnectionAttempt() {
    lastTcpAttemptMs = millis();
    tcpAttemptInProgress = true;
    tcpStatus = "TCP: connecting to " + tcpHost + ":" + tcpPort;
    new Thread(new Runnable() {
      public void run() {
        connectTcpInBackground();
      }
    }).start();
  }

  void connectTcpInBackground() {
    Socket socket = new Socket();
    try {
      socket.connect(new InetSocketAddress(tcpHost, tcpPort), tcpConnectTimeoutMs);
      pendingTcpSocket = socket;
    }
    catch (Exception error) {
      closeSocket(socket);
      tcpAttemptFailed = true;
    }
  }

  void closeSocket(Socket socket) {
    try {
      socket.close();
    }
    catch (Exception ignored) {
    }
  }
}
