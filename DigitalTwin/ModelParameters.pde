/**
 * @brief Every number that describes the PHYSICAL robot in this Digital Twin.
 *
 * Phase 2 asks you to update the sim with what Phase 1 found. Two sets are
 * kept so you can show what the update changed:
 *   nominalModel()    the template's ideal robot (unchanged assumptions)
 *   calibratedModel() your measured robot: put your Phase 1 results here
 * Press P in the simulator to switch. The active set is written into every
 * run's CSV and summary, so each result says which model produced it.
 *
 * The controller never reads these values. Like the real StampC3, it only
 * sees encoder counts and sensor readings, and it uses its OWN beliefs about
 * the wheels (Controller_c.config: wheel radii and separation). A difference
 * between the true geometry here and the controller's beliefs is exactly
 * what produces odometry (pose) error in the scan.
 */
class ModelParameters_c {
  String name;

  // ---- Geometry: the true robot, used by the physics ---------------------
  float wheelRadiusLeftMm;
  float wheelRadiusRightMm;
  float wheelSeparationMm;
  float encoderCountsPerRevolution;
  /** 1 = no slip. Below 1, the body turns less than the wheels say it should
   *  (wheel slip / scrub). Measure with a commanded 360 degree spin. */
  float rotationEfficiency;

  // ---- Motors -------------------------------------------------------------
  /** Wheel speed gained per PWM step above the deadband, in mm/s. */
  float motorGainMmPerSPerPwm;
  /** PWM magnitude that does not move the wheel at all. */
  float motorDeadbandPwm;
  /** Largest wheel speed, in mm/s (saturation). */
  float motorMaxSpeedMmPerS;
  /** First-order response time of the wheel speed, in ms (0 = instant). */
  float motorTimeConstantMs;
  /** Right wheel speed divided by left wheel speed at the same PWM. */
  float rightMotorScale;

  // ---- Surface sensors ----------------------------------------------------
  /** Sensor positions relative to the wheel-axle centre, in mm:
   *  ahead (+ forwards) and left (+ to the robot's left). DN1..DN5. */
  float[] sensorAheadMm = new float[5];
  float[] sensorLeftMm = new float[5];
  /** Reading on white paper and on black ink, per sensor. Higher = darker. */
  float[] sensorWhite = new float[5];
  float[] sensorBlack = new float[5];
  /** Spot size of each sensor (Gaussian sigma, mm). 0 = a single point. */
  float sensorFootprintSigmaMm;
  /** Random noise added to each reading (standard deviation, in counts). */
  float sensorNoiseSd;
  /** Age of a reading when the controller gets it, in ms (multiple of 10). */
  int sensorDelayMs;
  /** What the sensors see off the image: 1 = white, 0 = black. */
  float offImageReflectance;

  // ---- Placement ------------------------------------------------------------
  /** How far off the start cross you place the robot by hand (SD, mm and deg).
   *  This is what makes repeated runs differ. */
  float startPositionSdMm;
  float startHeadingSdDeg;

  /** @return A one-line description for logs and the status bar. */
  String describe() {
    return name + " (r " + nf(wheelRadiusLeftMm, 0, 2) + "/" + nf(wheelRadiusRightMm, 0, 2)
      + " mm, sep " + nf(wheelSeparationMm, 0, 1) + " mm, motor "
      + nf(motorGainMmPerSPerPwm, 0, 2) + " mm/s/PWM, dead " + nf(motorDeadbandPwm, 0, 0)
      + ", lag " + nf(motorTimeConstantMs, 0, 0) + " ms, spot " + nf(sensorFootprintSigmaMm, 0, 1)
      + " mm, noise " + nf(sensorNoiseSd, 0, 0) + ", delay " + sensorDelayMs + " ms, placement sd "
      + nf(startPositionSdMm, 0, 1) + " mm/" + nf(startHeadingSdDeg, 0, 1) + " deg)";
  }
}

/**
 * @brief The template's ideal robot: the assumptions the supplied Digital
 * Twin started with (MotorsModel, SurfaceSensorModel, Simulator defaults).
 * Keep this unchanged so you can compare "before" and "after" Phase 1.
 */
ModelParameters_c nominalModel() {
  ModelParameters_c p = new ModelParameters_c();
  p.name = "Nominal (template)";
  p.wheelRadiusLeftMm = 16.0;
  p.wheelRadiusRightMm = 16.0;
  p.wheelSeparationMm = 90.0;
  p.encoderCountsPerRevolution = 358.3;
  p.rotationEfficiency = 1.0;
  p.motorGainMmPerSPerPwm = 1.25;     // template: 0.25 mm/ms at PWM 200
  p.motorDeadbandPwm = 0;
  p.motorMaxSpeedMmPerS = 250;
  p.motorTimeConstantMs = 0;
  p.rightMotorScale = 1.0;
  // Template layout: all five sensors 45 mm from the centre at 45, 14, 0,
  // -14 and -45 degrees.
  float[] angles = {45, 14, 0, -14, -45};
  for (int i = 0; i < 5; i++) {
    p.sensorAheadMm[i] = 45 * cos(radians(angles[i]));
    p.sensorLeftMm[i] = 45 * sin(radians(angles[i]));
    p.sensorWhite[i] = 400;
    p.sensorBlack[i] = 2000;
  }
  p.sensorFootprintSigmaMm = 0;
  p.sensorNoiseSd = 0;
  p.sensorDelayMs = 0;
  p.offImageReflectance = 1.0;
  p.startPositionSdMm = 0;
  p.startHeadingSdDeg = 0;
  return p;
}

/**
 * @brief Our robot as measured. Replace every value marked PHASE 1 with your
 * own measurement (and keep a note of how you measured it for the report).
 * Values marked MEASURED come from our robot already.
 */
ModelParameters_c calibratedModel() {
  ModelParameters_c p = new ModelParameters_c();
  p.name = "Calibrated (Phase 1)";

  // Geometry
  p.wheelRadiusLeftMm = 16.0;          // PHASE 1: distance test along a ruler
  p.wheelRadiusRightMm = 16.0;         // PHASE 1: straight-line drift test
  p.wheelSeparationMm = 82.5;          // MEASURED: U-turn calibration (template said 90)
  p.encoderCountsPerRevolution = 358.3;
  p.rotationEfficiency = 1.0;          // PHASE 1: commanded 360 degree spin vs actual

  // Motors. These three together give the ~58 mm/s we measured at PWM 30
  // from our scan logs. PHASE 1: replace with your motor fit (speed vs PWM).
  p.motorGainMmPerSPerPwm = 2.88;      // ESTIMATE
  p.motorDeadbandPwm = 10;             // ESTIMATE (turns needed PWM 24 to start reliably)
  p.motorTimeConstantMs = 80;          // ESTIMATE
  p.motorMaxSpeedMmPerS = 500;
  p.rightMotorScale = 1.0;             // PHASE 1: equal PWM on both wheels, measure the curve

  // Surface sensors. Positions MEASURED from the underside photo (+-2 mm):
  //                 DN1   DN2   DN3   DN4    DN5
  float[] ahead = {28.0, 40.0, 41.0, 40.0,  28.0};
  float[] left  = {27.0,  9.0,  0.0, -9.0, -27.0};
  for (int i = 0; i < 5; i++) {
    p.sensorAheadMm[i] = ahead[i];
    p.sensorLeftMm[i] = left[i];
    p.sensorWhite[i] = 400;            // PHASE 1: each sensor's reading on white paper
    p.sensorBlack[i] = 2000;           // PHASE 1: each sensor's reading on black ink
  }
  p.sensorFootprintSigmaMm = 3.0;      // ESTIMATE from deblurring our scans. PHASE 1: edge scan
  p.sensorNoiseSd = 0;                 // PHASE 1: SD of readings with the robot still
  p.sensorDelayMs = 0;                 // PHASE 1: dashboard "Auto-calibrate" fits this
  p.offImageReflectance = 1.0;         // PHASE 1: reading on your table / bench

  // Placement: put the robot on the start cross 10 times and measure.
  p.startPositionSdMm = 1.0;           // ESTIMATE
  p.startHeadingSdDeg = 0.5;           // ESTIMATE
  return p;
}
