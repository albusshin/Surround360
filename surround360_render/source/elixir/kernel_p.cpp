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

class ProjectSphericalKernelCPUExtracted : public elixir::Kernel{

public:
  ProjectSphericalKernelCPUExtracted(size_t eqr_width,
                                     size_t eqr_height,
                                     string &camera_rig_path,
                                     int camIdx)
    : eqr_width_(eqr_width),
      eqr_height_(eqr_height),
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

  /*
    Accept:
    [0]: frame_col_mat

    Output:
      p_mat
  */

  std::unordered_map<std::string, void *> execute(
    std::vector<elixir::Data> dataList) {

    // dataList contains
    assert(dataList.size() == 1);
    assert(dataList[0]->data.size() == 1);

    cv::Mat& frame_col_mat = *(cv::Mat *) dataList[0]["frame_col_mat"];

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

    output_mat = new cv::Mat(output_image_height, output_image_width, cv_madetype);

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
  std::unique_ptr<RigDescription> rig_;
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
