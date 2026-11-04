/**
 * Image-rasterisation controller: a port of the StampC3 ImageRaster
 * Controller.cpp / Controller.h, kept line-for-line where possible so a change
 * on one side is easy to copy to the other. Names match the C++ code.
 *
 * Differences from the StampC3 version:
 *  - the scan log is an ArrayList (no memory limit) and also records the
 *    robot's TRUE pose, which only a simulation can know;
 *  - telemetry is a dummy call (the Simulator owns TCP), as in the template.
 */

/** Robot keeps facing +x and drives rows alternately forwards and in reverse. */
final int PATTERN_BACK_AND_FORTH = 0;
/** Robot turns 90 degrees, shifts, and turns 90 degrees again at each row end. */
final int PATTERN_UTURN = 1;

/** @brief Raster scan settings, as in RasterConfig_t (Controller.h). */
class RasterConfig_c {
  int pattern = PATTERN_BACK_AND_FORTH;
  float margin_mm = 50.0;
  int rows = 17;
  float row_length_mm = 160.0;
  float row_pitch_mm = 7.0;
  float scan_pwm = 30.0;
  float turn_pwm = 30.0;
  float turn_min_pwm = 24.0;
  int side = 1;
  float sensor_lead_mm = 40.0;
  // The controller's BELIEFS about the wheels (used for its odometry).
  float wheel_radius_left_mm = 16.0;
  float wheel_radius_right_mm = 16.0;
  float wheel_separation_mm = 82.5;

  RasterConfig_c copy() {
    RasterConfig_c c = new RasterConfig_c();
    c.pattern = pattern; c.margin_mm = margin_mm; c.rows = rows;
    c.row_length_mm = row_length_mm; c.row_pitch_mm = row_pitch_mm;
    c.scan_pwm = scan_pwm; c.turn_pwm = turn_pwm; c.turn_min_pwm = turn_min_pwm;
    c.side = side; c.sensor_lead_mm = sensor_lead_mm;
    c.wheel_radius_left_mm = wheel_radius_left_mm;
    c.wheel_radius_right_mm = wheel_radius_right_mm;
    c.wheel_separation_mm = wheel_separation_mm;
    return c;
  }

  String describe() {
    return (pattern == PATTERN_UTURN ? "U-turns" : "back and forth") + ", " + rows + " rows x "
      + nf(row_length_mm, 0, 0) + " mm, pitch " + nf(row_pitch_mm, 0, 1) + " mm, PWM "
      + nf(scan_pwm, 0, 0);
  }
}

/** @brief One logged sample (ScanSample_t plus the simulation's true pose). */
class ScanSample_c {
  long t_ms;
  float x, y, theta;            // the robot's own estimate (odometry)
  float[] dn = new float[5];    // DN1..DN5 as the controller saw them
  int state;
  int row;
  float true_x, true_y, true_theta;  // simulation only: where it really was
}

class Controller_c {
  // ---- Modes (the numbers are written into the log/CSV) -------------------
  final int WAITING = 0, SETTLING = 1, SCANNING_ROW = 2, TURNING_OUT = 3, SHIFTING = 4,
    TURNING_IN = 5, FINISHED = 6, STOPPED = 7, OVERRUN = 8, MARGIN = 9;

  // ---- Timing --------------------------------------------------------------
  final int UPDATE_INTERVAL_MS = 10;
  final int TELEMETRY_INTERVAL_MS = 50;
  final int LOG_INTERVAL_MS = 20;
  final int START_DELAY_MS = 1000;
  final int SETTLE_MS = 300;
  final int RAMP_MS = 300;

  // ---- Control gains (keep aligned with Controller.h) -------------------------
  final float HEADING_GAIN = 60.0;
  final float MAX_HEADING_CORRECTION = 15.0;
  final float TURN_GAIN = 40.0;
  final float TURN_TOLERANCE_RAD = 0.03;
  final float TURN_BOOST_STEP = 3.0;
  final float TURN_BOOST_MAX = 25.0;
  final int TURN_STALL_MS = 250;
  final float LINE_LOOKAHEAD_MM = 35.0;
  final float MAX_LINE_ANGLE_RAD = 0.45;

  // ---- Safety ----------------------------------------------------------------
  final int TURN_TIMEOUT_MS = 6000;
  final int STRAIGHT_TIMEOUT_MS = 30000;

  /** @brief The robot's own encoder odometry (reset at the start of every scan). */
  EncoderOdometry_c odometry = new EncoderOdometry_c();
  RasterConfig_c config = new RasterConfig_c();
  ArrayList<ScanSample_c> scan_log = new ArrayList<ScanSample_c>();

  int signal = 0;
  int mode = WAITING;
  int next_mode = WAITING;
  long mode_start_ms = 0;
  long settle_ms = 0;
  int row = 0;
  float target_heading = 0;
  float segment_start_x = 0, segment_start_y = 0;
  int pass_dir = 1;
  float row_y = 0;
  float turn_boost = 0;
  float turn_check_theta = 0;
  long turn_check_ms = 0;
  int run_id = 0;
  long log_tick = 0;
  boolean start_requested = false;
  boolean stop_requested = false;
  String message = "Ready";

  TaskTimer_c update_timer = new TaskTimer_c(UPDATE_INTERVAL_MS);
  TaskTimer_c telemetry_timer = new TaskTimer_c(TELEMETRY_INTERVAL_MS);
  TaskTimer_c log_timer = new TaskTimer_c(LOG_INTERVAL_MS);

  /** @brief Returns to WAITING. Sends no motor command (same as the template). */
  void reset() {
    signal = 0;
    mode = WAITING;
    next_mode = WAITING;
    mode_start_ms = 0;
    settle_ms = 0;
    row = 0;
    target_heading = 0;
    segment_start_x = 0;
    segment_start_y = 0;
    pass_dir = 1;
    row_y = 0;
    turn_boost = 0;
    turn_check_theta = 0;
    turn_check_ms = 0;
    start_requested = false;
    stop_requested = false;
    update_timer.resetTimer(0);
    telemetry_timer.resetTimer(0);
    log_timer.resetTimer(0);
  }

  int getSignal() { return signal; }

  /** @brief Non-zero starts a scan (if idle); zero returns to waiting. */
  void setSignal(int requested_signal) {
    if (requested_signal == 0) {
      reset();
      return;
    }
    if (signal == 0) requestStart();
  }

  void requestStart() { start_requested = true; }
  void requestStop() { stop_requested = true; }

  boolean isRunning() { return mode != WAITING && mode != FINISHED && mode != STOPPED; }
  int getMode() { return mode; }
  int getRow() { return row; }
  int getRunId() { return run_id; }
  String getMessage() { return message; }

  String getModeName() {
    String[] names = {"WAITING", "SETTLING", "SCANNING_ROW", "TURNING_OUT", "SHIFTING",
      "TURNING_IN", "FINISHED", "STOPPED", "OVERRUN", "MARGIN"};
    return mode >= 0 && mode < names.length ? names[mode] : "UNKNOWN";
  }

  /** @brief Applies new settings (same limits as the robot). */
  boolean setConfig(RasterConfig_c c) {
    if (isRunning()) return false;
    if (c.rows < 1 || c.rows > 150) return false;
    if (!(c.row_length_mm >= 20 && c.row_length_mm <= 1500)) return false;
    if (!(c.row_pitch_mm >= 2 && c.row_pitch_mm <= 300)) return false;
    if (!(c.scan_pwm >= 5 && c.scan_pwm <= 120)) return false;
    if (!(c.turn_pwm >= 5 && c.turn_pwm <= 120)) return false;
    if (!(c.turn_min_pwm >= 0 && c.turn_min_pwm <= c.turn_pwm)) return false;
    if (c.side != 1 && c.side != -1) return false;
    if (c.pattern != PATTERN_BACK_AND_FORTH && c.pattern != PATTERN_UTURN) return false;
    if (!(c.margin_mm >= 0 && c.margin_mm <= 200)) return false;
    if (!(c.sensor_lead_mm >= 0 && c.sensor_lead_mm <= 80)) return false;
    if (!(c.wheel_radius_left_mm >= 5 && c.wheel_radius_left_mm <= 50)) return false;
    if (!(c.wheel_radius_right_mm >= 5 && c.wheel_radius_right_mm <= 50)) return false;
    if (!(c.wheel_separation_mm >= 20 && c.wheel_separation_mm <= 200)) return false;
    config = c;
    message = "Settings updated";
    return true;
  }

  /**
   * @brief Copies the robot's settings from its dashboard (/config JSON),
   * so the twin scans exactly what the robot scans.
   */
  boolean applyRobotConfig(JSONObject j) {
    RasterConfig_c c = config.copy();
    if (j.hasKey("pattern")) c.pattern = round(j.getFloat("pattern")) == 1 ? PATTERN_UTURN : PATTERN_BACK_AND_FORTH;
    if (j.hasKey("margin")) c.margin_mm = j.getFloat("margin");
    if (j.hasKey("rows")) c.rows = round(j.getFloat("rows"));
    if (j.hasKey("row_len")) c.row_length_mm = j.getFloat("row_len");
    if (j.hasKey("pitch")) c.row_pitch_mm = j.getFloat("pitch");
    if (j.hasKey("scan_pwm")) c.scan_pwm = j.getFloat("scan_pwm");
    if (j.hasKey("turn_pwm")) c.turn_pwm = j.getFloat("turn_pwm");
    if (j.hasKey("turn_min")) c.turn_min_pwm = j.getFloat("turn_min");
    if (j.hasKey("side")) c.side = j.getFloat("side") < 0 ? -1 : 1;
    if (j.hasKey("lead")) c.sensor_lead_mm = j.getFloat("lead");
    if (j.hasKey("r_left")) c.wheel_radius_left_mm = j.getFloat("r_left");
    if (j.hasKey("r_right")) c.wheel_radius_right_mm = j.getFloat("r_right");
    if (j.hasKey("sep")) c.wheel_separation_mm = j.getFloat("sep");
    return setConfig(c);
  }

  /** @brief Runs one 10 ms acquisition, control, logging and telemetry cycle. */
  void update(Robot_c robot) {
    long now = robot.getMillis();
    if (!update_timer.isReady(now)) return;
    update_timer.resetTimer(now);

    // 1) Sense
    robot.getSurfaceSensors();
    if (robot.getEncoders()) {
      odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
    }
    robot.getPose();

    // 2) Requests (the StampC3 also checks its button here)
    if (stop_requested) {
      stop_requested = false;
      start_requested = false;
      if (isRunning()) {
        robot.setMotorPWM(0, 0);
        mode = STOPPED;
        signal = 0;
        message = "Stopped by user";
      }
    }
    if (start_requested) {
      start_requested = false;
      if (!isRunning()) beginScan(robot, now);
    }

    // 3) Act
    runRaster(robot, now);

    // 4) Log (same rule as the robot: turns and run-ins at half rate)
    if (log_timer.isReady(now)) {
      log_timer.resetTimer(now);
      if (isRunning() && mode != SETTLING) {
        boolean turning = (mode == TURNING_OUT || mode == TURNING_IN || mode == MARGIN);
        log_tick++;
        if (!turning || (log_tick % 2) == 0) logSample(robot, now);
      }
    }

    // 5) Telemetry
    if (telemetry_timer.isReady(now)) {
      telemetry_timer.resetTimer(now);
      publishTelemetry(robot);
    }
  }

  void beginScan(Robot_c robot, long now) {
    robot.setMotorPWM(0, 0);
    odometry.setLeftWheelRadiusMm(config.wheel_radius_left_mm);
    odometry.setRightWheelRadiusMm(config.wheel_radius_right_mm);
    odometry.wheelSeparationMm = config.wheel_separation_mm;
    odometry.reset(0, 0, 0);

    scan_log.clear();
    run_id++;
    signal = 1;
    row = 0;
    row_y = 0;
    pass_dir = 1;
    target_heading = rowHeading(0);
    message = "Scanning";
    robot.playTone(1200, 80);
    settleThen(robot, now, START_DELAY_MS, SCANNING_ROW);
  }

  void runRaster(Robot_c robot, long now) {
    switch (mode) {
    case WAITING:
      return;

    case FINISHED:
    case STOPPED:
      robot.setMotorPWM(0, 0);
      return;

    case SETTLING:
      robot.setMotorPWM(0, 0);
      if (now - mode_start_ms >= settle_ms) enterMode(next_mode, now);
      return;

    case MARGIN:
      runBackAndForth(robot, now);
      return;

    case SCANNING_ROW:
    case OVERRUN: {
      if (config.pattern == PATTERN_BACK_AND_FORTH) {
        runBackAndForth(robot, now);
        return;
      }
      float travelled = distanceFromSegmentStart();
      if (mode == SCANNING_ROW && travelled >= config.row_length_mm) {
        if (row + 1 >= config.rows) {
          finish(robot, "Scan complete");
          return;
        }
        mode = OVERRUN;
      }
      if (mode == OVERRUN && travelled >= config.row_length_mm + 2.0 * config.sensor_lead_mm) {
        endOfRow(robot, now);
        return;
      }
      if (now - mode_start_ms > STRAIGHT_TIMEOUT_MS) {
        fail(robot, "Row timed out: robot barely moving? Raise scan PWM");
        return;
      }
      driveOnRowLine(robot, rowHeading(row), 1, now);
      return;
    }

    case TURNING_OUT:
    case TURNING_IN: {
      float err = headingError();
      boolean aligned = abs(err) < TURN_TOLERANCE_RAD;
      boolean timed_out = (now - mode_start_ms) > TURN_TIMEOUT_MS;
      if (aligned || timed_out) {
        if (timed_out) message = "Turn timed out: raise turn min PWM (deadband?)";
        if (mode == TURNING_OUT) {
          settleThen(robot, now, SETTLE_MS, SHIFTING);
        } else {
          row++;
          row_y = config.side * row * config.row_pitch_mm;
          settleThen(robot, now, SETTLE_MS, SCANNING_ROW);
        }
        return;
      }
      turnInPlace(robot, err, now);
      return;
    }

    case SHIFTING: {
      float next_line_y = config.side * (row + 1) * config.row_pitch_mm;
      boolean reached = (config.side > 0) ? (odometry.pose.y >= next_line_y)
        : (odometry.pose.y <= next_line_y);
      if (reached) {
        target_heading = rowHeading(row + 1);
        settleThen(robot, now, SETTLE_MS, TURNING_IN);
        return;
      }
      if (now - mode_start_ms > STRAIGHT_TIMEOUT_MS) {
        fail(robot, "Shift timed out: robot barely moving? Raise scan PWM");
        return;
      }
      driveStraight(robot, config.scan_pwm, now);
      return;
    }
    }
  }

  void endOfRow(Robot_c robot, long now) {
    target_heading = config.side * HALF_PI;
    settleThen(robot, now, SETTLE_MS, TURNING_OUT);
  }

  void runBackAndForth(Robot_c robot, long now) {
    float x = odometry.pose.x;
    float L = config.row_length_mm;
    float m = config.margin_mm;
    boolean last = (row + 1 >= config.rows);

    boolean done = (pass_dir > 0) ? (x >= (last ? L : L + m))
      : (x <= (last ? 0.0 : -m));
    if (done) {
      if (last) {
        finish(robot, "Scan complete");
        return;
      }
      row++;
      pass_dir = -pass_dir;
      row_y = config.side * row * config.row_pitch_mm;
      settleThen(robot, now, SETTLE_MS, MARGIN);
      return;
    }

    if (now - mode_start_ms > STRAIGHT_TIMEOUT_MS) {
      fail(robot, "Row timed out: robot barely moving? Raise scan PWM");
      return;
    }

    mode = (x >= 0 && x <= L) ? SCANNING_ROW : MARGIN;
    driveOnRowLine(robot, 0, pass_dir, now);
  }

  void driveOnRowLine(Robot_c robot, float line_heading, int dir, long now) {
    float ramp = (float)(now - mode_start_ms) / (float)RAMP_MS;
    if (ramp > 1) ramp = 1;
    float base = config.scan_pwm * ramp * dir;

    float e = row_y - odometry.pose.y;
    float c = cos(line_heading) >= 0 ? 1.0 : -1.0;
    float steer = atan(e / LINE_LOOKAHEAD_MM);
    steer = constrain(steer, -MAX_LINE_ANGLE_RAD, MAX_LINE_ANGLE_RAD);
    target_heading = line_heading + dir * c * steer;

    float max_corr = 5.0 + (MAX_HEADING_CORRECTION - 5.0) * ramp;
    float turn = HEADING_GAIN * headingError();
    turn = constrain(turn, -max_corr, max_corr);
    robot.setMotorPWM(base - turn, base + turn);
  }

  void enterMode(int new_mode, long now) {
    mode = new_mode;
    mode_start_ms = now;
    segment_start_x = odometry.pose.x;
    segment_start_y = odometry.pose.y;
    if (new_mode == TURNING_OUT || new_mode == TURNING_IN) {
      turn_boost = 0;
      turn_check_theta = odometry.pose.theta;
      turn_check_ms = now;
    }
  }

  void settleThen(Robot_c robot, long now, long duration_ms, int then) {
    robot.setMotorPWM(0, 0);
    settle_ms = duration_ms;
    next_mode = then;
    enterMode(SETTLING, now);
  }

  void finish(Robot_c robot, String why) {
    robot.setMotorPWM(0, 0);
    mode = FINISHED;
    signal = 0;
    message = why;
    robot.playTone(880, 300);
  }

  void fail(Robot_c robot, String why) {
    robot.setMotorPWM(0, 0);
    mode = STOPPED;
    signal = 0;
    message = why;
    robot.playTone(300, 500);
  }

  void driveStraight(Robot_c robot, float base_pwm, long now) {
    float ramp = (float)(now - mode_start_ms) / (float)RAMP_MS;
    if (ramp > 1) ramp = 1;
    float forward = base_pwm * ramp;
    float turn = HEADING_GAIN * headingError();
    turn = constrain(turn, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
    robot.setMotorPWM(forward - turn, forward + turn);
  }

  void turnInPlace(Robot_c robot, float heading_error, long now) {
    if (now - turn_check_ms >= TURN_STALL_MS) {
      if (abs(odometry.pose.theta - turn_check_theta) < 0.01) {
        turn_boost = min(turn_boost + TURN_BOOST_STEP, TURN_BOOST_MAX);
      }
      turn_check_theta = odometry.pose.theta;
      turn_check_ms = now;
    }
    float lo = config.turn_min_pwm + turn_boost;
    float hi = max(config.turn_pwm, lo);
    float magnitude = TURN_GAIN * abs(heading_error);
    magnitude = constrain(magnitude, lo, hi);
    float direction = (heading_error > 0) ? 1.0 : -1.0;
    robot.setMotorPWM(-direction * magnitude, direction * magnitude);
  }

  float headingError() {
    return wrapAngle(target_heading - odometry.pose.theta);
  }

  float distanceFromSegmentStart() {
    float dx = odometry.pose.x - segment_start_x;
    float dy = odometry.pose.y - segment_start_y;
    return sqrt(dx * dx + dy * dy);
  }

  void logSample(Robot_c robot, long now) {
    ScanSample_c s = new ScanSample_c();
    s.t_ms = now;
    s.x = odometry.pose.x;
    s.y = odometry.pose.y;
    s.theta = odometry.pose.theta;
    for (int i = 0; i < 5; i++) s.dn[i] = robot.surface.reading[i];
    s.state = mode;
    s.row = row;
    s.true_x = robot.odometry.pose.x;
    s.true_y = robot.odometry.pose.y;
    s.true_theta = robot.odometry.pose.theta;
    scan_log.add(s);
  }

  float wrapAngle(float angle) {
    return atan2(sin(angle), cos(angle));
  }

  float rowHeading(int row_index) {
    return (row_index % 2 == 0) ? 0.0 : PI;
  }

  /** @brief Placeholder matching the StampC3 telemetry task (see template). */
  void publishTelemetry(Robot_c robot) {
  }
}
