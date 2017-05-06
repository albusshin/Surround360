#ifndef SURROUND360_KERNELS_ELIXIR
#define SURROUND360_KERNELS_ELIXIR

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

#include "kernel.h"

typedef int i32;
using namespace std;
using namespace surround360;

class KernelI : public elixir::Kernel {
public:
  KernelI(string videoFilename,
          int frameNumber)
    : videoFilename_(videoFilename),
      frameNumber_(frameNumber) {}

  KernelI *clone() override {
    return new KernelI(videoFileName_, frameNumber_);
  };

  void updateToNextLayer() override {
    frameNumber++;
  }

  unordered_map<string, void *> execute(
    vector<elixir::Data> dataList);

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
    vector<elixir::Data> dataList);

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

  unordered_map<string, void *> execute(
    vector<elixir::Data> dataList);

private:
  string camera_rig_path_;
  string flow_algo_;

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

  unordered_map<string, void *> execute(
    vector<elixir::Data> dataList);

private:
  int camImageHeight_;
  unique_ptr<RigDescription> rig_;
  float fov_horizontal_radians_;
  int overlap_image_width_;
  int num_novel_views_;

  size_t eqr_width;
  size_t eqr_height;
  string camera_rig_path;
  string flow_algo;
  float zero_parallax_dist;
  float interpupilary_dist;

  unique_ptr<NovelViewGenerator> novel_view_gen_;
  unique_ptr<LazyNovelViewBuffer> lazy_view_buffer_;
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

  unordered_map<string, void *> execute(
    vector<elixir::Data> dataList);

private:
  unique_ptr<RigDescription> rig_;
  float zeroParallaxNovelViewShiftPixels_;
  int camImageWidth_;
  int camImageHeight_;

  size_t eqr_width;
  size_t eqr_height;
  string &camera_rig_path;
  string &flow_algo;
  float zero_parallax_dist;
  float interpupilary_dist;
  bool left;
};

#endif /* SURROUND360_KERNELS_ELIXIR */
