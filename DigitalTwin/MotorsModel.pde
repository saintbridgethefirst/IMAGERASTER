/**
 * @brief Paired motor-response model for the simulated robot.
 * @details Each wheel: no motion inside the deadband, a linear gain above it,
 * saturation at the top speed, and a first-order lag. The template's ideal
 * linear motor is the special case deadband 0, lag 0 (nominalModel()).
 * All values come from ModelParameters_c so Phase 1 results go in one place.
 */
class MotorsModel_c {
  MotorModel_c left = new MotorModel_c();
  MotorModel_c right = new MotorModel_c();

  /** @brief Stores requested left and right PWM values after limiting them. */
  void setPWM(float leftPwm, float rightPwm) {
    left.setPWM(leftPwm);
    right.setPWM(rightPwm);
  }

  /** @brief Advances both wheel speeds by one simulation step. */
  void update(ModelParameters_c p, int dtMs) {
    left.update(p, 1.0, dtMs);
    right.update(p, p.rightMotorScale, dtMs);
  }

  /** @brief Stops both wheels immediately (used when a run is reset). */
  void stop() {
    left.pwm = 0; left.speedMmPerMs = 0;
    right.pwm = 0; right.speedMmPerMs = 0;
  }
}

/** @brief Response model for one motor. */
class MotorModel_c {
  float pwm = 0;
  /** Wheel surface speed in mm per ms (positive = forwards). */
  float speedMmPerMs = 0;

  /** @brief Limits and stores a requested PWM value in [-200, +200]. */
  void setPWM(float value) { pwm = constrain(value, -200, 200); }

  /** @return The speed this PWM settles to, in mm/ms. */
  float steadySpeed(ModelParameters_c p, float scale) {
    float magnitude = abs(pwm) - p.motorDeadbandPwm;
    if (magnitude <= 0) return 0;
    float mmPerS = min(magnitude * p.motorGainMmPerSPerPwm * scale, p.motorMaxSpeedMmPerS);
    return (pwm > 0 ? 1 : -1) * mmPerS / 1000.0;
  }

  /** @brief Moves the speed towards its steady value (first-order lag). */
  void update(ModelParameters_c p, float scale, int dtMs) {
    float target = steadySpeed(p, scale);
    if (p.motorTimeConstantMs <= 0) {
      speedMmPerMs = target;
    } else {
      float alpha = 1.0 - exp(-dtMs / p.motorTimeConstantMs);
      speedMmPerMs += (target - speedMmPerMs) * alpha;
    }
  }
}
