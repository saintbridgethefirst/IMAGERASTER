/** @brief Cached five-channel surface-sensor data exposed to the controller. */
class RobotSurface_c {
  float[] reading = {400, 400, 400, 400, 400};
}

/**
 * @brief Model of one downward-facing reflectance sensor.
 * @details Position, spot size and response come from ModelParameters_c.
 * The surface image is 1 pixel per mm, centred on its world offset, with
 * image y inverted relative to world y (as in the template).
 *
 * Changes from the template's ideal point sensor:
 *  - real positions (ahead/left of the axle), not a 45 mm circle;
 *  - a Gaussian spot (footprint) instead of a single pixel;
 *  - per-sensor white and black levels (offset and gain);
 *  - noise (added by Robot_c) and delay (handled by Robot_c).
 */
class SurfaceSensorModel_c {
  String name;
  int index;
  float localX;   // ahead of the axle, mm
  float localY;   // to the left, mm

  // Footprint kernel: offsets (mm) and weights, rebuilt when sigma changes.
  float kernelSigma = -1;
  float[] kx, ky, kw;

  SurfaceSensorModel_c(String nameIn, int indexIn) {
    name = nameIn;
    index = indexIn;
  }

  /** @brief Copies this sensor's position from the model parameters. */
  void configure(ModelParameters_c p) {
    localX = p.sensorAheadMm[index];
    localY = p.sensorLeftMm[index];
    if (p.sensorFootprintSigmaMm != kernelSigma) buildKernel(p.sensorFootprintSigmaMm);
  }

  void buildKernel(float sigma) {
    kernelSigma = sigma;
    if (sigma <= 0.05) {
      kx = new float[] {0};
      ky = new float[] {0};
      kw = new float[] {1};
      return;
    }
    int r = ceil(2.0 * sigma);
    int n = (2 * r + 1) * (2 * r + 1);
    kx = new float[n];
    ky = new float[n];
    kw = new float[n];
    float total = 0;
    int k = 0;
    for (int dy = -r; dy <= r; dy++) {
      for (int dx = -r; dx <= r; dx++) {
        kx[k] = dx;
        ky[k] = dy;
        kw[k] = exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma));
        total += kw[k];
        k++;
      }
    }
    for (int i = 0; i < n; i++) kw[i] /= total;
  }

  /** @return World x of this sensor for a robot pose. */
  float worldX(float robotX, float robotTheta) {
    return robotX + localX * cos(robotTheta) - localY * sin(robotTheta);
  }

  /** @return World y of this sensor for a robot pose. */
  float worldY(float robotY, float robotTheta) {
    return robotY + localX * sin(robotTheta) + localY * cos(robotTheta);
  }

  /**
   * @brief Noise-free reading for the robot's TRUE pose.
   * @return A count between this sensor's white and black levels.
   */
  float sample(PImage image, float surfaceOffsetX, float surfaceOffsetY,
    float robotX, float robotY, float robotTheta, ModelParameters_c p) {
    float wx = worldX(robotX, robotTheta);
    float wy = worldY(robotY, robotTheta);
    float reflectance = 0;
    for (int k = 0; k < kw.length; k++) {
      reflectance += kw[k] * surfaceReflectance(image, surfaceOffsetX, surfaceOffsetY,
        wx + kx[k], wy + ky[k], p.offImageReflectance);
    }
    return p.sensorBlack[index] + (p.sensorWhite[index] - p.sensorBlack[index]) * reflectance;
  }

  /** @brief Draws a small sensor marker in the robot frame. */
  void draw(float reading, ModelParameters_c p) {
    pushStyle();
    float span = p.sensorBlack[index] - p.sensorWhite[index];
    float darkness = span != 0 ? constrain((reading - p.sensorWhite[index]) / span, 0, 1) : 0;
    fill(220 - 200 * darkness);
    stroke(20);
    rectMode(CENTER);
    rect(localX, localY, 3, 3);
    popStyle();
  }
}

/**
 * @brief Reflectance of the surface at a world point: 1 = white, 0 = black.
 * @details Uses luminance directly (no Processing colour mode involved), so
 * grey prints map linearly.
 */
float surfaceReflectance(PImage image, float surfaceOffsetX, float surfaceOffsetY,
  float worldX, float worldY, float offImage) {
  int pixelX = floor(image.width / 2.0 + worldX - surfaceOffsetX);
  int pixelY = floor(image.height / 2.0 - worldY + surfaceOffsetY);
  if (pixelX < 0 || pixelX >= image.width || pixelY < 0 || pixelY >= image.height) {
    return offImage;
  }
  int c = image.pixels[pixelY * image.width + pixelX];
  float r = (c >> 16) & 0xFF;
  float g = (c >> 8) & 0xFF;
  float b = c & 0xFF;
  return (0.299 * r + 0.587 * g + 0.114 * b) / 255.0;
}
