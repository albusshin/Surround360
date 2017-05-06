#include "surround360_kernels.h"

typedef int i32;

void KernelC::new_frame_info(
  int camImageWidth, int camImageHeight) {
  camImageWidth_ = camImageWidth;
  camImageHeight_ = camImageHeight;
  std::cout << "[Concat]\t"
            << "new_frame_info: "
            << "camImageWidth = "
            << camImageWidth
            << "camImageHeight = "
            << camImageHeight;
  const int numCams = 14;
  const float cameraRingRadius = rig_->getRingRadius();
  const float camFovHorizontalDegrees =
    2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
  const float fovHorizontalRadians = toRadians(camFovHorizontalDegrees);
  const float overlapAngleDegrees =
    (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
  const int overlapImageWidth =
    float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);

  const float v =
    atanf(zero_parallax_dist_ / (interpupilary_dist_ / 2.0f));
  const float psi =
    asinf(sinf(v) * (interpupilary_dist_ / 2.0f) / cameraRingRadius);
  const float vergeAtInfinitySlabDisplacement =
    psi * (float(camImageWidth) / fovHorizontalRadians);
  const float theta = -M_PI / 2.0f + v + psi;
  zeroParallaxNovelViewShiftPixels_ =
    float(eqr_width_) * (theta / (2.0f * M_PI));
  if (!left_) {
    zeroParallaxNovelViewShiftPixels_ *= -1;
  }
}
