#include "surround360_kernels.h"

typedef int i32;

/*
  Accept:
  [0]: frame_col_mat

  Output:
  p_mat
*/

std::unordered_map<std::string, void *> KernelP::execute(
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
