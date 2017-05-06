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

/*
  Accept:
  [0]: p_mat

  [1]: p_mat

  [2]: prev_overlap_image_l_
  [2]: prev_overlap_image_r_
  [2]: prev_frame_flow_l_to_r_
  [2]: prev_frame_flow_r_to_l_

  Output:
  left_flow
  right_flow
  prev_frame_flow_l_to_r_
  prev_frame_flow_r_to_l_
  prev_overlap_image_l_
  prev_overlap_image_r_
*/
std::unordered_map<std::string, void *> KernelF::execute (
  std::vector<elixir::Data *>& dataList) {

  /* Magic numbers fest */
  assert(dataList.size() == 3);
  assert(dataList[0].data.size() == 1);
  cv::Mat& left_input = *(cv::Mat *) dataList[0].data["p_mat"];
  int camImageWidthL = left_input.cols;
  int camImageHeightL = left_input.rows;

  assert(dataList[1].data.size() == 1);
  cv::Mat& right_input = *(cv::Mat *) dataList[1].data["p_mat"];
  int camImageWidthR = right_input.cols;
  int camImageHeightR = right_input.rows;

  assert(camImageWidthL == camImageWidthR);
  assert(camImageHeightL == camImageHeightR);

  new_frame_info(camImageWidthL, camImageHeightL);

  assert(dataList[1].data.size() == 6);
  cv::Mat prev_overlap_image_l_ = *(cv::Mat *) dataList[2].data["prev_overlap_image_l_"];
  cv::Mat prev_overlap_image_r_ = *(cv::Mat *) dataList[2].data["prev_overlap_image_r_"];
  cv::Mat prev_frame_flow_l_to_r_ = *(cv::Mat *) dataList[2].data["prev_frame_flow_l_to_r_"];
  cv::Mat prev_frame_flow_r_to_l_ = *(cv::Mat *) dataList[2].data["prev_frame_flow_r_to_l_"];

  assert(overlap_image_width_ != -1);

  size_t output_image_width = overlap_image_width_;
  size_t output_image_height = camImageHeight_;

  std::cout << "T: left_input = "
            << left_input.cols
            << " * "
            << left_input.rows
            << " * "
            << left_input.channels()
            << std::endl;

  cv::Mat left_overlap_input =
    left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                        overlap_image_width_, left_input.rows));
  cv::Mat right_overlap_input =
    right_input(cv::Rect(0, 0,
                         overlap_image_width_, right_input.rows));

  std::cout << "T: left_overlap_input = "
            << left_overlap_input.cols
            << " * "
            << left_overlap_input.rows
            << " * "
            << left_overlap_input.channels()
            << std::endl;

  novel_view_gen_->prepare(left_overlap_input, right_overlap_input,
                           prev_frame_flow_l_to_r_, prev_frame_flow_r_to_l_,
                           prev_overlap_image_l_, prev_overlap_image_r_);

  cv::Mat *new_prev_overlap_image_l_ = new cv::Mat();
  cv::Mat *new_prev_overlap_image_r_ = new cv::Mat();
  cv::Mat *new_prev_frame_flow_l_to_r_ = new cv::Mat();
  cv::Mat *new_prev_frame_flow_r_to_l_ = new cv::Mat();

  *new_prev_frame_flow_l_to_r_ = novel_view_gen_->getFlowLtoR();
  *new_prev_frame_flow_r_to_l_ = novel_view_gen_->getFlowRtoL();
  left_overlap_input.copyTo(*new_prev_overlap_image_l_);
  right_overlap_input.copyTo(*new_prev_overlap_image_r_);

  cv::Mat *left_flow = new cv::Mat();
  cv::Mat *right_flow = new cv::Mat();

  *left_flow = novel_view_gen_->getFlowLtoR();
  *right_flow = novel_view_gen_->getFlowRtoL();

  std::unordered_map<std::string, void *> outputData;
  outputData["left_flow"] = ((void *) left_flow);
  outputData["right_flow"] = ((void *) right_flow);
  outputData["prev_frame_flow_l_to_r_"] = ((void *) new_prev_frame_flow_l_to_r_);
  outputData["prev_frame_flow_r_to_l_"] = ((void *) new_prev_frame_flow_r_to_l_);
  outputData["prev_overlap_image_l_"] = ((void *) new_prev_overlap_image_l_);
  outputData["prev_overlap_image_r_"] = ((void *) new_prev_overlap_image_r_);

  return outputData;
}
