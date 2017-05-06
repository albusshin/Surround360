#include "surround360_kernels.h"

typedef int i32;

void KernelR::new_frame_info(int camImageWidth, int camImageHeight) {
  camImageHeight_ = camImageHeight;

  const int numCams = 14;
  const float cameraRingRadius = rig_->getRingRadius();
  const float camFovHorizontalDegrees =
    2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
  fov_horizontal_radians_ = toRadians(camFovHorizontalDegrees);
  const float overlapAngleDegrees =
    (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
  overlap_image_width_ = float(camImageWidth) *
    (overlapAngleDegrees / camFovHorizontalDegrees);
  num_novel_views_ =
    camImageWidth - overlap_image_width_; // per image pair

  const float v =
    atanf(zero_parallax_dist_ / (interpupilary_dist_ / 2.0f));
  const float psi =
    asinf(sinf(v) * (interpupilary_dist_ / 2.0f) / cameraRingRadius);
  const float vergeAtInfinitySlabDisplacement =
    psi * (float(camImageWidth) / fov_horizontal_radians_);
  const float theta = -M_PI / 2.0f + v + psi;
  const float zeroParallaxNovelViewShiftPixels =
    float(eqr_width_) * (theta / (2.0f * M_PI));

  int currChunkX = 0;
  lazy_view_buffer_.reset(new LazyNovelViewBuffer(eqr_width_ / numCams,
                                                  camImageHeight));
  for (int nvIdx = 0; nvIdx < num_novel_views_; ++nvIdx) {
    const float shift = float(nvIdx) / float(num_novel_views_);
    const float slabShift =
      float(camImageWidth) * 0.5f - float(num_novel_views_ - nvIdx);

    for (int v = 0; v < camImageHeight; ++v) {
      lazy_view_buffer_->warpL[currChunkX][v] =
        Point3f(slabShift + vergeAtInfinitySlabDisplacement, v, shift);
      lazy_view_buffer_->warpR[currChunkX][v] =
        Point3f(slabShift - vergeAtInfinitySlabDisplacement, v, shift);
    }
    ++currChunkX;
  }
}
