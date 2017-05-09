#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"

typedef int i32;
using namespace elixir;

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

std::unordered_map<std::string, void *> KernelR::execute (
  std::vector<elixir::Data *>& dataList) {

  logger << "[KernelR T"
         << elixir::Worker::getWorkerId()
         << "]\t"
         << "execute()"
         << endl;

  time_t t = time(0);
  printf("[Kernel-R-start]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  assert(dataList.size() == 3);
  assert(dataList[0]->data.size() == 1);
  assert(dataList[1]->data.size() == 1);
  assert(dataList[2]->data.size() == 6);

  vector<cv::Mat>& left_inputs = *(vector<cv::Mat> *) dataList[0]->data["p_mats"];
  vector<cv::Mat>& right_inputs = *(vector<cv::Mat> *) dataList[1]->data["p_mats"];
  vector<cv::Mat>& left_flows = *(vector<cv::Mat> *) dataList[2]->data["left_flows"];
  vector<cv::Mat>& right_flows = *(vector<cv::Mat> *) dataList[2]->data["right_flows"];

  assert(left_inputs.size() == right_inputs.size());
  assert(left_inputs.size() == left_flows.size());
  assert(left_inputs.size() == right_flows.size());

  vector<cv::Mat>* chunkLs = new vector<cv::Mat>();
  vector<cv::Mat>* chunkRs = new vector<cv::Mat>();

  for (int frameNum = 0; frameNum < left_inputs.size(); ++frameNum) {
    cv::Mat& left_input = left_inputs[frameNum];
    cv::Mat& right_input = right_inputs[frameNum];
    int camImageWidthL = left_input.cols;
    int camImageHeightL = left_input.rows;
    int camImageWidthR = right_input.cols;
    int camImageHeightR = right_input.rows;

    assert(camImageWidthL == camImageWidthR);
    assert(camImageHeightL == camImageHeightR);
    cv::Mat& left_flow = left_flows[frameNum];
    cv::Mat& right_flow = right_flows[frameNum];

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

    cv::Mat chunkL;
    cv::Mat chunkR;
    chunkL = lazyNovelChunksLR.first;
    chunkR = lazyNovelChunksLR.second;

    chunkLs->push_back(chunkL);
    chunkRs->push_back(chunkR);
  }

  std::unordered_map<std::string, void *> outputData;
  outputData["chunkLs"] = ((void *) chunkLs);
  outputData["chunkRs"] = ((void *) chunkRs);

  t = time(0);
  printf("[Kernel-R-end]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  return outputData;
}
