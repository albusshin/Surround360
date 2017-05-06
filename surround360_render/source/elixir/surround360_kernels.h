#ifndef SURROUND360_KERNELS_ELIXIR
#define SURROUND360_KERNELS_ELIXIR

#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "optical_flow/NovelView.h"
#include "util/MathUtil.h"
// #include "source/scanner_kernels/surround360.pb.h"
#include "optical_flow/NovelView.h"
#include "kernel.h"

#include <opencv2/video.hpp>
#include <string>
#include <sstream>
#include <ctime>
#include <unordered_map>
#include <assert.h>

#include "kernel.h"

typedef int i32;
using namespace std;
using namespace surround360;
using namespace surround360::optical_flow;
using namespace surround360::math_util;

class KernelI : public elixir::Kernel {
public:
  KernelI(string videoFilename,
          int frameNumber)
    : videoFilename_(videoFilename),
      frameNumber_(frameNumber) {}

  KernelI *clone() override {
    return new KernelI(videoFilename_, frameNumber_);
  };

  void updateToNextLayer() override {
    frameNumber_++;
  }

  unordered_map<string, void *> execute(
    vector<elixir::Data>& dataList) override {
    assert(dataList.empty());

    cv::VideoCapture cap(videoFilename_);
    if(!cap.isOpened()){
      std::cout << "I: Cannot open video file "
                << videoFilename_
                << std::endl;
      assert(false);
    }

    cap.set(CV_CAP_PROP_POS_FRAMES, frameNumber_);

    cv::Mat *frame_col_mat = new cv::Mat();
    cap >> *frame_col_mat;

    std::unordered_map<std::string, void *> outputData;
    outputData["frame_col_mat"] = ((void *) frame_col_mat);

    return outputData;
  }

private:
  string videoFilename_;
  int frameNumber_;
};

class KernelP : public elixir::Kernel {

public:
  KernelP(size_t eqr_width,
          size_t eqr_height,
          string &camera_rig_path,
          int camIdx)
    : eqr_width_(eqr_width),
      eqr_height_(eqr_height),
      camera_rig_path_(camera_rig_path),
      camIdx_(camIdx) {

    // Initialize camera rig
    rig_.reset(new RigDescription(camera_rig_path));

    hRadians_ = 2 * approximateFov(rig_->rigSideOnly, false);
    vRadians_ = 2 * approximateFov(rig_->rigSideOnly, true);

    const Camera& camera = rig_->rigSideOnly[camIdx_];

    // the negative sign here is so the camera array goes clockwise
    const int numCameras = 14;
    float direction = -float(camIdx_) / float(numCameras) * 2.0f * M_PI;
    leftAngle_ = direction + hRadians_ / 2;
    rightAngle_ = direction - hRadians_ / 2;
    topAngle_ = vRadians_ / 2;
    bottomAngle_ = -vRadians_ / 2;
  }

  KernelP *clone() override {
    return new KernelP(eqr_width_,
                       eqr_height_,
                       camera_rig_path_,
                       camIdx_);
  };

  void updateToNextLayer() override {}

  /*
    Accept:
    [0]: frame_col_mat

    Output:
    p_mat
  */
  unordered_map<string, void *> execute(
    vector<elixir::Data>& dataList) override {

    // dataList contains
    assert(dataList.size() == 1);
    assert(dataList[0].data.size() == 1);

    cv::Mat& frame_col_mat = *(cv::Mat *) dataList[0].data["frame_col_mat"];

    size_t output_image_width = eqr_width_ * hRadians_ / (2 * M_PI);
    size_t output_image_height = eqr_height_ * vRadians_ / M_PI;
    size_t output_image_size = output_image_width * output_image_height * 4;

    int channels = 4;
    int cv_type = CV_8U;
    int cv_madetype = CV_MAKETYPE(cv_type, channels);


    std::cout << "P: output = "
              << frame_col_mat.cols
              << " * "
              << frame_col_mat.rows
              << " * "
              << frame_col_mat.channels()
              << std::endl;

    cv::Mat tmp;
    cv::cvtColor(frame_col_mat, tmp, CV_BGR2BGRA);
    std::cout << "after cvtColor()" << std::endl;

    cv::Mat *output_mat = new cv::Mat(output_image_height, output_image_width, cv_madetype);

    surround360::warper::bicubicRemapToSpherical(
      *output_mat, //dst
      tmp, //src
      rig_->rigSideOnly[camIdx_],
      leftAngle_,
      rightAngle_,
      topAngle_,
      bottomAngle_);

    std::unordered_map<std::string, void *> outputData;

    outputData["p_mat"] = ((void *) output_mat);
    std::cout << "after bicubicRemapToSpherical" << std::endl;

    return outputData;
  }

private:
  unique_ptr<RigDescription> rig_;
  string camera_rig_path_;
  size_t eqr_width_;
  size_t eqr_height_;

  float hRadians_;
  float vRadians_;

  int camIdx_;
  float leftAngle_;
  float rightAngle_;
  float topAngle_;
  float bottomAngle_;
};

class KernelF : public elixir::Kernel {

public:
  KernelF(string &camera_rig_path,
          string &flow_algo)
    : camera_rig_path_(camera_rig_path),
      flow_algo_(flow_algo)
    {

      rig_.reset(new RigDescription(camera_rig_path));

      overlap_image_width_ = -1;
      novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(flow_algo));

    }

  KernelF *clone() override {
    return new KernelF(camera_rig_path_, flow_algo_);
  };

  void updateToNextLayer() override {}

  void new_frame_info(int camImageWidth, int camImageHeight);

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
  unordered_map<string, void *> execute(
    vector<elixir::Data>& dataList) override {

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

private:
  string camera_rig_path_;
  string flow_algo_;
  int camImageHeight_;

  std::unique_ptr<RigDescription> rig_;
  int overlap_image_width_;
  std::unique_ptr<surround360::optical_flow::NovelViewGenerator> novel_view_gen_;

};

class KernelR : public elixir::Kernel{
public:
  KernelR(
    size_t eqr_width,
    size_t eqr_height,
    string &camera_rig_path,
    string &flow_algo,
    float zero_parallax_dist,
    float interpupilary_dist)
    : eqr_width_(eqr_width),
      eqr_height_(eqr_height),
      camera_rig_path_(camera_rig_path),
      flow_algo_(flow_algo),
      zero_parallax_dist_(zero_parallax_dist),
      interpupilary_dist_(interpupilary_dist)
    {

      rig_.reset(new RigDescription(camera_rig_path));

      overlap_image_width_ = -1;
      novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(flow_algo));
    }

  KernelR *clone() override {
    return new KernelR(eqr_width_,
                       eqr_height_,
                       camera_rig_path_,
                       flow_algo_,
                       zero_parallax_dist_,
                       interpupilary_dist_);
  };

  void updateToNextLayer() override {}

  void new_frame_info(int camImageWidth, int camImageHeight);

/*
  Accept:
  [0]: frame_col_mat

  Output:
  p_mat
*/

  unordered_map<string, void *> execute(
    vector<elixir::Data>& dataList) override {
    assert(dataList.size() == 3);
    assert(dataList[0].data.size() == 1);
    assert(dataList[1].data.size() == 1);
    assert(dataList[2].data.size() == 6);

    cv::Mat& left_input = *(cv::Mat *) dataList[0].data["p_mat"];
    cv::Mat& right_input = *(cv::Mat *) dataList[1].data["p_mat"];
    int camImageWidthL = left_input.cols;
    int camImageHeightL = left_input.rows;
    int camImageWidthR = right_input.cols;
    int camImageHeightR = right_input.rows;

    assert(camImageWidthL == camImageWidthR);
    assert(camImageHeightL == camImageHeightR);

    cv::Mat& left_flow = *(cv::Mat *) dataList[2].data["left_flow"];
    cv::Mat& right_flow = *(cv::Mat *) dataList[2].data["right_flow"];

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
    outputData["chunkL"] = ((void *) chunkL);
    outputData["chunkR"] = ((void *) chunkR);

    return outputData;

  }

private:
  int camImageHeight_;
  unique_ptr<RigDescription> rig_;
  float fov_horizontal_radians_;
  int overlap_image_width_;
  int num_novel_views_;

  size_t eqr_width_;
  size_t eqr_height_;
  string camera_rig_path_;
  string flow_algo_;
  float zero_parallax_dist_;
  float interpupilary_dist_;

  unique_ptr<surround360::optical_flow::NovelViewGenerator> novel_view_gen_;
  unique_ptr<surround360::optical_flow::LazyNovelViewBuffer> lazy_view_buffer_;
};

class KernelC : public elixir::Kernel{
public:
  KernelC(
    size_t eqr_width,
    size_t eqr_height,
    string &camera_rig_path,
    string &flow_algo,
    float zero_parallax_dist,
    float interpupilary_dist,
    bool left)
    : eqr_width_(eqr_width),
      eqr_height_(eqr_height),
      camera_rig_path_(camera_rig_path),
      flow_algo_(flow_algo),
      zero_parallax_dist_(zero_parallax_dist),
      interpupilary_dist_(interpupilary_dist),
      left_(left) {

    rig_.reset(new RigDescription(camera_rig_path));
  }

  KernelC *clone() override {
    return new KernelC(eqr_width_,
                       eqr_height_,
                       camera_rig_path_,
                       flow_algo_,
                       zero_parallax_dist_,
                       interpupilary_dist_,
                       left_);
  };

  void updateToNextLayer() override {}

  void new_frame_info(int camImageWidth, int camImageHeight);

/*
  Accept:
  [0]: chunkL / chunkR
  ...
  [13]: chunkL / chunkR

  Output:
  pano
*/
  unordered_map<string, void *> execute(
    vector<elixir::Data>& dataList) override {

    string chunkKey;
    if (left_) {
      chunkKey = "chunkL";
    } else {
      chunkKey = "chunkR";
    }

    i32 num_chunks = dataList.size();
    assert(num_chunks == 14);

    cv::Mat& tmp_input_chunk = *(cv::Mat *) dataList[0].data[chunkKey];

    int camImageWidth = tmp_input_chunk.cols;
    int camImageHeight = tmp_input_chunk.rows;

    new_frame_info(camImageWidth, camImageHeight);

    size_t output_image_width = camImageWidth_ * num_chunks;
    size_t output_image_height = camImageHeight_;
    output_image_width += (output_image_width % 2);
    output_image_height += (output_image_height % 2);
    size_t output_image_size =
      output_image_width * output_image_height * 3;

    std::vector<cv::Mat> pano_chunks(num_chunks, Mat());
    for (i32 c = 0; c < num_chunks; ++c) {
      cv::Mat& input_chunk = *(cv::Mat *) dataList[c].data[chunkKey];
      assert(dataList[c].data.size() == 2);
      cv::cvtColor(input_chunk, pano_chunks[c], CV_BGRA2BGR);
    }
    cv::Mat *pano = new cv::Mat();

    *pano = stackHorizontal(pano_chunks);
    *pano = offsetHorizontalWrap(*pano, zeroParallaxNovelViewShiftPixels_);

    std::unordered_map<std::string, void *> outputData;
    outputData["pano"] = ((void *) pano);

    return outputData;
  }

private:
  unique_ptr<RigDescription> rig_;
  float zeroParallaxNovelViewShiftPixels_;
  int camImageWidth_;
  int camImageHeight_;

  size_t eqr_width_;
  size_t eqr_height_;
  string &camera_rig_path_;
  string &flow_algo_;
  float zero_parallax_dist_;
  float interpupilary_dist_;
  bool left_;
};

#endif /* SURROUND360_KERNELS_ELIXIR */
