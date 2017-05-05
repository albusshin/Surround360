#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "optical_flow/NovelView.h"
#include "util/MathUtil.h"
#include "source/scanner_kernels/surround360.pb.h"
#include "optical_flow/NovelView.h"

#include <opencv2/video.hpp>
#include <string>
#include <sstream>
#include <ctime>
#include <unordered_map>
#include <assert.h>

typedef int i32;

class RenderStereoPanoramaChunkKernelCPUExtracted : public elixir::Kernel{
public:
  RenderStereoPanoramaChunkKernelCPUExtracted(
    size_t eqr_width,
    size_t eqr_height,
    string &camera_rig_path,
    string &flow_algo,
    float zero_parallax_dist,
    float interpupilary_dist)
    : eqr_width(eqr_width),
      eqr_height(eqr_height),
      camera_rig_path(camera_rig_path),
      flow_algo(flow_algo),
      zero_parallax_dist(zero_parallax_dist),
      interpupilary_dist(interpupilary_dist)
    {

    rig_.reset(new RigDescription(camera_rig_path));

    overlap_image_width_ = -1;
    novel_view_gen_.reset(
      new NovelViewGeneratorAsymmetricFlow(flow_algo));
  }

  void new_frame_info(int camImageWidth, int camImageHeight) {
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
      atanf(zero_parallax_dist / (interpupilary_dist / 2.0f));
    const float psi =
      asinf(sinf(v) * (interpupilary_dist / 2.0f) / cameraRingRadius);
    const float vergeAtInfinitySlabDisplacement =
      psi * (float(camImageWidth) / fov_horizontal_radians_);
    const float theta = -M_PI / 2.0f + v + psi;
    const float zeroParallaxNovelViewShiftPixels =
      float(eqr_width) * (theta / (2.0f * M_PI));

    int currChunkX = 0;
    lazy_view_buffer_.reset(new LazyNovelViewBuffer(eqr_width / numCams,
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

  /*
    Accept:
    [0]: p_mat
    [1]: p_mat

    [2]: left_flow
    [2]: right_flow

    Output:
      chunkL 
      chunkR 
  */
  std::unordered_map<std::string, void *> execute(
    std::vector<elixir::Data> dataList) {

    assert(dataList.size() == 3);
    assert(dataList[0].size() == 1);
    assert(dataList[1].size() == 1);
    assert(dataList[2].size() == 6);

    cv::Mat& left_input = *(cv::Mat *) dataList[0]->data["p_mat"];
    cv::Mat& right_input = *(cv::Mat *) dataList[1]->data["p_mat"];
    int camImageWidthL = left_flow.cols;
    int camImageHeightL = left_flow.rows;

    cv::Mat& left_flow = *(cv::Mat *) dataList[2]->data["left_flow"];
    cv::Mat& right_flow = *(cv::Mat *) dataList[2]->data["right_flow"];
    int camImageWidthR = right_flow.cols;
    int camImageHeightR = right_flow.rows;

    assert(camImageWidthL == camImageWidthR);
    assert(camImageHeightL == camImageHeightR);

    new_frame_info(camImageWidthL, camImageHeightL);

    assert(overlap_image_width_ != -1);

    size_t output_image_width = num_novel_views_;
    size_t output_image_height = camImageHeight_;
    size_t output_image_size =
      output_image_width * output_image_height * 4;

    cv::Mat left_overlap_input =
      left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                          overlap_image_width_, left_input.rows));
    cv::Mat right_overlap_input =
      right_input(cv::Rect(0, 0,
                           overlap_image_width_, right_input.rows));


    // Initialize NovelViewGenerator with images and flow since we are
    // bypassing the prepare method
    novel_view_gen_->setImageL(left_overlap_input);
    novel_view_gen_->setImageR(right_overlap_input);
    novel_view_gen_->setFlowLtoR(left_flow);
    novel_view_gen_->setFlowRtoL(right_flow);
    std::pair<Mat, Mat> lazyNovelChunksLR =
      novel_view_gen_->combineLazyNovelViews(*lazy_view_buffer_.get());

    cv::Mat *chunkL = new cv::Mat();
    cv::Mat *chunkR = new cv::Mat();
    *chunkL = lazyNovelChunksLR.first;
    *chunkR = lazyNovelChunksLR.second;

    std::unordered_map<std::string, void *> outputData;
    outputData["chunkL"]((void *) chunkL);
    outputData["chunkR"]((void *) chunkR);

    return outputData;
  }

private:
  int camImageHeight_;
  std::unique_ptr<RigDescription> rig_;
  float fov_horizontal_radians_;
  int overlap_image_width_;
  int num_novel_views_;

  size_t eqr_width;
  size_t eqr_height;
  string camera_rig_path;
  string flow_algo;
  float zero_parallax_dist;
  float interpupilary_dist;

  std::unique_ptr<NovelViewGenerator> novel_view_gen_;
  std::unique_ptr<LazyNovelViewBuffer> lazy_view_buffer_;
}
