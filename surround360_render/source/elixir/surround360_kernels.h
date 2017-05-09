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
#include "nullbuf.h"

#define DEBUG

#define logger cout
//#define logger null_stream

typedef int i32;
using namespace std;
using namespace surround360;
using namespace surround360::optical_flow;
using namespace surround360::math_util;

class KernelI : public elixir::Kernel {
public:
  KernelI(cv::VideoCapture* cap,
          int startFrameIndex,
          size_t batchSize)
    : cap_(cap),
      startFrameIndex_(startFrameIndex),
      batchSize_(batchSize) {

    assert(cap->isOpened());
    cap->set(CV_CAP_PROP_POS_FRAMES, startFrameIndex_);
    logger << "[KernelI]\t"
           << "ctor()"
           << "cap == "
           << cap
           << endl;
  }

  KernelI *clone() override {
    return new KernelI(cap, startFrameIndex_, batchSize_);
  };

  void updateToNextLayer() override {
    startFrameIndex_ += batchSize_;
  }

  unordered_map<string, void *> execute(
    vector<elixir::Data *>& dataList);

private:
  int startFrameIndex_;
  size_t batchSize_;
  cv::VideoCapture *cap_;
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
    logger << "[KernelP]\t"
           << "ctor()"
           << "camIdx == "
           << camIdx
           << endl;

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
    vector<elixir::Data *>& dataList);

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
      logger << "[KernelF]\t"
             << "ctor()"
             << endl;

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

  unordered_map<string, void *> execute(
    vector<elixir::Data *>& dataList);

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
      logger << "[KernelR]\t"
             << "ctor()"
             << endl;

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

  unordered_map<string, void *> execute(
    vector<elixir::Data *>& dataList);

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

    logger << "[KernelC]\t"
           << "ctor()"
           << endl;

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

  unordered_map<string, void *> execute(
    vector<elixir::Data *>& dataList);

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
