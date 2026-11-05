/**
 * @brief Top-level model of the robot, excluding the environment.
 * @details Coordinates the motor, surface-sensor and odometry models. Its
 * public methods mirror the parts of the StampC3 Robot_c that Controller_c
 * uses, so the raster controller runs here with the same calls.
 *
 * The physics (true pose, encoders, sensor readings) uses ModelParameters_c.
 * The controller only ever sees whole encoder counts and sensor readings.
 */
class Robot_c {
  /** @brief Radius used only to draw the robot body, in mm. */
  final float bodyRadiusMm = 48.5;
  /** @brief Longest sensor delay supported, in 10 ms steps. */
  final int HISTORY_STEPS = 64;

  ModelParameters_c model;
  long simulationMillis = 0;
  PImage surfaceImage;
  float surfaceOffsetX;
  float surfaceOffsetY;
  RobotSurface_c surface = new RobotSurface_c();
  MotorsModel_c motors = new MotorsModel_c();
  /** @brief TRUE pose and encoder counts (the physics). */
  OdometryModel_c odometry = new OdometryModel_c();
  SurfaceSensorModel_c[] sensors = new SurfaceSensorModel_c[5];

  // Sensor history for the reading delay: one entry per physics step.
  float[][] history = new float[HISTORY_STEPS][5];
  int historyHead = 0;
  int historyCount = 0;

  /**
   * @param modelIn Physical robot parameters (nominal or calibrated).
   * @param surfaceIn Environment image sampled by the five surface sensors.
   * @param surfaceOffsetXIn Surface-image centre x in world millimetres.
   * @param surfaceOffsetYIn Surface-image centre y in world millimetres.
   */
  Robot_c(ModelParameters_c modelIn, PImage surfaceIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    surfaceImage = surfaceIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    for (int i = 0; i < 5; i++) sensors[i] = new SurfaceSensorModel_c("DN" + (i + 1), i);
    setModel(modelIn);
    resetForRun(0, 0, 0, 0, 0);
  }

  /** @brief Puts a different sheet under the robot (1 px/mm, centred at the offset). */
  void setSurface(PImage surfaceIn, float offsetX, float offsetY) {
    surfaceImage = surfaceIn;
    surfaceOffsetX = offsetX;
    surfaceOffsetY = offsetY;
  }

  /** @brief Switches the physical parameters (for example nominal/calibrated). */
  void setModel(ModelParameters_c modelIn) {
    model = modelIn;
    for (int i = 0; i < 5; i++) sensors[i].configure(model);
  }

  /** @return Current simulated time in milliseconds. */
  long getMillis() { return simulationMillis; }

  /** @brief Advances the simulated clock. */
  void advanceTime(int dtMs) { simulationMillis += dtMs; }

  /** @brief Gives the controller the (possibly delayed, noisy) readings. */
  void getSurfaceSensors() {
    if (historyCount == 0) sampleSensors();
    int stepsBack = min(model.sensorDelayMs / 10, historyCount - 1);
    int index = (historyHead - 1 - stepsBack + HISTORY_STEPS) % HISTORY_STEPS;
    for (int i = 0; i < 5; i++) surface.reading[i] = history[index][i];
  }

  /** @brief Compatibility with the StampC3 API: counts are always fresh here. */
  boolean getEncoders() { return true; }

  /** @return Whole encoder counts, as the 3Pi+ reports them. */
  long getLeftEncoderCount() { return (long) Math.floor(odometry.encoders.left); }
  long getRightEncoderCount() { return (long) Math.floor(odometry.encoders.right); }

  /** @brief Compatibility method; the true pose updates during physics steps. */
  void getPose() { }

  /** @brief Stores requested left and right motor PWM values. */
  void setMotorPWM(float leftPwm, float rightPwm) {
    motors.setPWM(leftPwm, rightPwm);
  }

  float getLeftMotorPWM() { return motors.left.pwm; }
  float getRightMotorPWM() { return motors.right.pwm; }

  /** @brief The StampC3 beeps; the twin stays quiet. */
  void playTone(int frequency, int durationMs) { }

  /** @brief Puts the robot at a pose, stopped, with fresh sensor history. */
  void resetForRun(float x, float y, float theta, float leftEncoder, float rightEncoder) {
    odometry.reset(x, y, theta, leftEncoder, rightEncoder);
    motors.stop();
    historyCount = 0;
    historyHead = 0;
    sampleSensors();
  }

  /** @brief Advances motor response, true motion, and the sensor readings. */
  void updatePhysics(int dtMs) {
    motors.update(model, dtMs);
    float leftDistance = motors.left.speedMmPerMs * dtMs;
    float rightDistance = motors.right.speedMmPerMs * dtMs;
    odometry.update(leftDistance, rightDistance, model);
    sampleSensors();
  }

  /** @brief Samples all five sensors at the true pose into the history. */
  void sampleSensors() {
    RobotPose_c pose = odometry.pose;
    float[] row = history[historyHead];
    for (int i = 0; i < 5; i++) {
      float value = sensors[i].sample(surfaceImage, surfaceOffsetX, surfaceOffsetY,
        pose.x, pose.y, pose.theta, model);
      if (model.sensorNoiseSd > 0) value += randomGaussian() * model.sensorNoiseSd;
      row[i] = max(0, value);
    }
    historyHead = (historyHead + 1) % HISTORY_STEPS;
    historyCount = min(historyCount + 1, HISTORY_STEPS);
  }

  /** @return The current five readings as the controller sees them. */
  float[] sensorReadings() { return surface.reading; }

  void draw() {
    RobotPose_c pose = odometry.pose;
    pushMatrix();
    translate(pose.x, pose.y);
    rotate(pose.theta);
    fill(190, 205, 220, 200);
    stroke(40);
    strokeWeight(2);
    ellipse(0, 0, bodyRadiusMm, bodyRadiusMm);
    stroke(40, 100, 220);
    strokeWeight(3);
    line(0, 0, bodyRadiusMm * .55, 0);
    noStroke();
    fill(40, 100, 220);
    triangle(bodyRadiusMm * .75, 0, bodyRadiusMm * .5, bodyRadiusMm * .12,
      bodyRadiusMm * .5, -bodyRadiusMm * .12);
    // wheels
    fill(40);
    rectMode(CENTER);
    rect(0, model.wheelSeparationMm / 2.0, 2 * model.wheelRadiusLeftMm, 6);
    rect(0, -model.wheelSeparationMm / 2.0, 2 * model.wheelRadiusRightMm, 6);
    strokeWeight(1);
    for (int i = 0; i < sensors.length; i++) sensors[i].draw(surface.reading[i], model);
    popMatrix();
  }
}
