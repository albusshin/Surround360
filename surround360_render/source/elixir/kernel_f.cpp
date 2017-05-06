#include "surround360_kernels.h"

typedef int i32;

void KernelF::new_frame_info(int camImageWidth, int camImageHeight) {
  camImageHeight_ = camImageHeight;
  const int numCams = 14;
  const float cameraRingRadius = rig_->getRingRadius();
  const float camFovHorizontalDegrees =
    2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
  const float fovHorizontalRadians = toRadians(camFovHorizontalDegrees);
  const float overlapAngleDegrees =
    (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
  overlap_image_width_ =
    float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);
}