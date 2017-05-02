#include "render/RigDescription.h"
#include "optical_flow/NovelView.h"
#include "util/MathUtil.h"
#include "source/scanner_kernels/surround360.pb.h"

#include <tuple>

typedef int i32;

namespace surround360 {

  using namespace optical_flow;
  using namespace math_util;

  class TemporalOpticalFlowKernelCPUExtracted {
  public:
    TemporalOpticalFlowKernelCPUExtracted(
      const surround360::proto::TemporalOpticalFlowArgs &args) {

      args_ = args;
      work_item_size_ = work_item_size;

      rig_.reset(new RigDescription(args_.camera_rig_path()));

      overlap_image_width_ = -1;
      novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(args_.flow_algo()));

      reset();
    }

    void reset() {
      prev_frame_flow_l_to_r_ = cv::Mat();
      prev_frame_flow_r_to_l_ = cv::Mat();
      prev_overlap_image_l_ = cv::Mat();
      prev_overlap_image_r_ = cv::Mat();
    }

    // TODO: must this function be invoked before adding a new frame?
    void new_frame_info(int camImageWidth, int camImageHeight) {
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

    std::tuple<cv::Mat, cv::Mat>& execute(cv::Mat& left_frame_col_mat,
                                          cv::Mat& right_frame_col_mat) {
      assert(overlap_image_width_ != -1);

      size_t output_image_width = overlap_image_width_;
      size_t output_image_height = frame_info_.height();

      cv::Mat left_overlap_input =
        left_frame_col_mat(cv::Rect(left_frame_col_mat.cols - overlap_image_width_, 0,
                                    overlap_image_width_, left_frame_col_mat.rows));
      cv::Mat right_overlap_input =
        right_frame_col_mat(cv::Rect(0, 0,
                                     overlap_image_width_, right_frame_col_mat.rows));

      novel_view_gen_->prepare(left_overlap_input, right_overlap_input,
                               prev_frame_flow_l_to_r_, prev_frame_flow_r_to_l_,
                               prev_overlap_image_l_, prev_overlap_image_r_);

      //  NOTE txin: This is to copy the state for next frame.
      left_overlap_input.copyTo(prev_overlap_image_l_);
      right_overlap_input.copyTo(prev_overlap_image_r_);
      prev_frame_flow_l_to_r_ = novel_view_gen_->getFlowLtoR();
      prev_frame_flow_r_to_l_ = novel_view_gen_->getFlowRtoL();

      left_flow = novel_view_gen_->getFlowLtoR();
      right_flow = novel_view_gen_->getFlowRtoL();
      return make_tuple<cv::Mat, cv::Mat>(left_flow, right_flow);
    }
  }

private:
  surround360::proto::TemporalOpticalFlowArgs args_;
  std::unique_ptr<RigDescription> rig_;
  int overlap_image_width_;

  std::unique_ptr<NovelViewGenerator> novel_view_gen_;
  cv::Mat prev_frame_flow_l_to_r_;
  cv::Mat prev_frame_flow_r_to_l_;
  cv::Mat prev_overlap_image_l_;
  cv::Mat prev_overlap_image_r_;
};
