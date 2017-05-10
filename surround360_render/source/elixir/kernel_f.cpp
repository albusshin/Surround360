#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"

#include <sstream>

typedef int i32;
using namespace elixir;

static std::atomic_int filenameCounter(0);

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

  logger << "[KernelF T"
         << elixir::Worker::getWorkerId()
         << "]\t"
         << "execute()"
         << endl;

  time_t t = time(0);
  printf("[Kernel-F-start]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  /* Magic numbers fest */
  assert(dataList.size() == 3);
  assert(dataList[0]->data.size() == 1);
  assert(dataList[1]->data.size() == 1);
  assert(dataList[2]->data.size() == 6);

  vector<cv::Mat>& left_inputs = *(vector<cv::Mat> *) dataList[0]->data["p_mats"];
  vector<cv::Mat>& right_inputs = *(vector<cv::Mat> *) dataList[1]->data["p_mats"];
  cv::Mat& prev_overlap_image_l = *(cv::Mat *) dataList[2]->data["prev_overlap_image_l"];
  cv::Mat& prev_overlap_image_r = *(cv::Mat *) dataList[2]->data["prev_overlap_image_r"];
  cv::Mat& prev_frame_flow_l_to_r = *(cv::Mat *) dataList[2]->data["prev_frame_flow_l_to_r"];
  cv::Mat& prev_frame_flow_r_to_l = *(cv::Mat *) dataList[2]->data["prev_frame_flow_r_to_l"];

  assert(left_inputs.size() == right_inputs.size());

  vector<cv::Mat> *left_flows = new vector<cv::Mat>();
  vector<cv::Mat> *right_flows = new vector<cv::Mat>();
  cv::Mat *new_prev_overlap_image_l = new cv::Mat(prev_overlap_image_l);
  cv::Mat *new_prev_overlap_image_r = new cv::Mat(prev_overlap_image_r);
  cv::Mat *new_prev_frame_flow_l_to_r = new cv::Mat(prev_frame_flow_l_to_r);
  cv::Mat *new_prev_frame_flow_r_to_l = new cv::Mat(prev_frame_flow_r_to_l);

  for (int frameNum = 0; frameNum < left_inputs.size(); ++frameNum) {
    cv::Mat& left_input = left_inputs[frameNum];
    cv::Mat& right_input = right_inputs[frameNum];

    int camImageWidthL = left_input.cols;
    int camImageHeightL = left_input.rows;

    int camImageWidthR = right_input.cols;
    int camImageHeightR = right_input.rows;

    assert(camImageWidthL == camImageWidthR);
    assert(camImageHeightL == camImageHeightR);

    new_frame_info(camImageWidthL, camImageHeightL);

    assert(overlap_image_width_ != -1);

    size_t output_image_width = overlap_image_width_;
    size_t output_image_height = camImageHeight_;

    cv::Mat left_overlap_input =
      left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                          overlap_image_width_, left_input.rows));
    cv::Mat right_overlap_input =
      right_input(cv::Rect(0, 0,
                           overlap_image_width_, right_input.rows));

    novel_view_gen_->prepare(left_overlap_input,
                             right_overlap_input,
                             *new_prev_frame_flow_l_to_r,
                             *new_prev_frame_flow_r_to_l,
                             *new_prev_overlap_image_l,
                             *new_prev_overlap_image_r);

    *new_prev_frame_flow_l_to_r = novel_view_gen_->getFlowLtoR();
    *new_prev_frame_flow_r_to_l = novel_view_gen_->getFlowRtoL();
    left_overlap_input.copyTo(*new_prev_overlap_image_l);
    right_overlap_input.copyTo(*new_prev_overlap_image_r);

    cv::Mat left_flow;
    cv::Mat right_flow;

    left_flow = novel_view_gen_->getFlowLtoR();
    right_flow = novel_view_gen_->getFlowRtoL();

    vector<cv::Mat> left_flow_channels(2);
    cv::split(left_flow, left_flow_channels);

    vector<cv::Mat> right_flow_channels(2);
    cv::split(right_flow, right_flow_channels);

    logger << "[KernelF T"
           << elixir::Worker::getWorkerId()
           << "]\t"
           << "left_flow: "
           << left_flow.cols
           << " * "
           << left_flow.rows
           << " * "
           << left_flow.channels()
           << endl;

    std::stringstream ss;

    ss << "/home/ubuntu/o/kernel-F-leftflow-1-" << (filenameCounter++) << ".jpg";

    cv::imwrite(ss.str(), left_flow_channels[1]);

    ss.clear();

    ss << "/home/ubuntu/o/kernel-F-leftflow-0-" << (filenameCounter++) << ".jpg";

    cv::imwrite(ss.str(), left_flow_channels[0]);

    ss.clear();
    ss << "/home/ubuntu/o/kernel-F-rightflow-0-" << (filenameCounter++) << ".jpg";

    cv::imwrite(ss.str(), right_flow_channels[0]);

    ss.clear();
    ss << "/home/ubuntu/o/kernel-F-rightflow-1-" << (filenameCounter++) << ".jpg";

    cv::imwrite(ss.str(), right_flow_channels[1]);

    left_flows->push_back(left_flow);
    right_flows->push_back(right_flow);
  }

  std::unordered_map<std::string, void *> outputData;
  outputData["left_flows"] = ((void *) left_flows);
  outputData["right_flows"] = ((void *) right_flows);
  outputData["prev_overlap_image_l"] = ((void *) new_prev_overlap_image_l);
  outputData["prev_overlap_image_r"] = ((void *) new_prev_overlap_image_r);
  outputData["prev_frame_flow_l_to_r"] = ((void *) new_prev_frame_flow_l_to_r);
  outputData["prev_frame_flow_r_to_l"] = ((void *) new_prev_frame_flow_r_to_l);

  t = time(0);
  printf("[Kernel-F-end]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  return outputData;
}
