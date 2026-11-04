/** @brief Cached left and right encoder counts. */
class RobotEncoders_c {
  float left = 0;
  float right = 0;
}

/** @brief Robot pose in world coordinates: x/y in mm and heading in radians. */
class RobotPose_c {
  float x = 0;
  float y = 0;
  float theta = 0;
}

/**
 * @brief TRUE differential-drive motion of the simulated robot (the physics).
 * @details Integrates each wheel's travel over a fixed simulation step and
 * produces the encoder counts the real encoders would report. Uses the true
 * geometry in ModelParameters_c, which the controller does not know.
 */
class OdometryModel_c {
  RobotEncoders_c encoders = new RobotEncoders_c();
  RobotPose_c pose = new RobotPose_c();

  /** @brief Sets the complete pose and encoder state for a new run. */
  void reset(float x, float y, float theta, float leftEncoder, float rightEncoder) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    encoders.left = leftEncoder;
    encoders.right = rightEncoder;
  }

  /**
   * @brief Integrates one differential-drive motion increment.
   * @param leftDistance Left wheel surface travel in mm over the step.
   * @param rightDistance Right wheel surface travel in mm over the step.
   * @param p True robot geometry.
   */
  void update(float leftDistance, float rightDistance, ModelParameters_c p) {
    float distance = (leftDistance + rightDistance) / 2.0;
    float deltaTheta = p.rotationEfficiency * (rightDistance - leftDistance) / p.wheelSeparationMm;
    // Midpoint heading: the truth should be more exact than the robot's estimate.
    float midTheta = pose.theta + deltaTheta / 2.0;
    pose.x += distance * cos(midTheta);
    pose.y += distance * sin(midTheta);
    pose.theta += deltaTheta;
    encoders.left += leftDistance * p.encoderCountsPerRevolution / (TWO_PI * p.wheelRadiusLeftMm);
    encoders.right += rightDistance * p.encoderCountsPerRevolution / (TWO_PI * p.wheelRadiusRightMm);
  }
}

/**
 * @brief The robot's OWN pose estimate from encoder counts.
 * @details A line-for-line port of StampC3 OdometryModel.cpp (first-order
 * Euler, separate wheel radii), so the twin makes the same estimate, and the
 * same mistakes, as the real robot. Counts are whole numbers, as on the robot.
 */
class EncoderOdometry_c {
  float leftWheelRadiusMm = 16.0;
  float rightWheelRadiusMm = 16.0;
  float wheelSeparationMm = 90.0;
  float encoderCountsPerRevolution = 358.3;
  RobotPose_c pose = new RobotPose_c();
  boolean initialised = false;
  long previousLeftCount = 0;
  long previousRightCount = 0;

  void setLeftWheelRadiusMm(float mm) { leftWheelRadiusMm = mm; }
  void setRightWheelRadiusMm(float mm) { rightWheelRadiusMm = mm; }

  /** @brief Sets the starting pose and forgets the previous counts. */
  void reset(float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    initialised = false;
    previousLeftCount = 0;
    previousRightCount = 0;
  }

  /** @return True after a pose update; false for the first count baseline. */
  boolean update(long leftCount, long rightCount) {
    if (leftWheelRadiusMm <= 0 || rightWheelRadiusMm <= 0 || wheelSeparationMm <= 0) return false;
    if (!initialised) {
      previousLeftCount = leftCount;
      previousRightCount = rightCount;
      initialised = true;
      return false;
    }
    float deltaLeftCounts = leftCount - previousLeftCount;
    float deltaRightCounts = rightCount - previousRightCount;
    previousLeftCount = leftCount;
    previousRightCount = rightCount;

    float leftDistance = deltaLeftCounts * TWO_PI * leftWheelRadiusMm / encoderCountsPerRevolution;
    float rightDistance = deltaRightCounts * TWO_PI * rightWheelRadiusMm / encoderCountsPerRevolution;
    float distance = (leftDistance + rightDistance) / 2.0;
    float deltaTheta = (rightDistance - leftDistance) / wheelSeparationMm;

    // First-order Euler: old heading for both increments, then heading.
    pose.x += distance * cos(pose.theta);
    pose.y += distance * sin(pose.theta);
    pose.theta += deltaTheta;
    return true;
  }
}
