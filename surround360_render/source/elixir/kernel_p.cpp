#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"

#include <sstream>

typedef int i32;
using namespace elixir;

static std::atomic<int> filenameCounter = 0;
/*
  Accept:
  [0]: frame_col_mat

  Output:
  p_mat
*/

std::unordered_map<std::string, void *> KernelP::execute (
  std::vector<elixir::Data *>& dataList) {

  logger << "[KernelP T"
         << elixir::Worker::getWorkerId()
         << "]\t"
         << "execute()"
         << endl;

  time_t t = time(0);
  printf("[Kernel-P-start]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  // dataList contains
  assert(dataList.size() == 1);
  assert(dataList[0]->data.size() == 1);

  vector<cv::Mat>& frame_col_mats =
    *(vector<cv::Mat> *) dataList[0]->data["frame_col_mats"];

  vector<cv::Mat> *p_mats = new vector<cv::Mat>();
  for (cv::Mat& frame_col_mat : frame_col_mats) {

    size_t output_image_width = eqr_width_ * hRadians_ / (2 * M_PI);
    size_t output_image_height = eqr_height_ * vRadians_ / M_PI;
    size_t output_image_size = output_image_width * output_image_height * 4;

    int channels = 4;
    int cv_type = CV_8U;
    int cv_madetype = CV_MAKETYPE(cv_type, channels);

    logger << "P: output = "
           << frame_col_mat.cols
           << " * "
           << frame_col_mat.rows
           << " * "
           << frame_col_mat.channels()
           << std::endl;

    cv::Mat tmp;
    cv::cvtColor(frame_col_mat, tmp, CV_BGR2BGRA);
    logger << "after cvtColor()" << std::endl;

    cv::Mat output_mat(output_image_height, output_image_width, cv_madetype);

    surround360::warper::bicubicRemapToSpherical(
      output_mat, //dst
      tmp, //src
      rig_->rigSideOnly[camIdx_],
      leftAngle_,
      rightAngle_,
      topAngle_,
      bottomAngle_);

    stringstream ss;
    ss <<  "/home/ubuntu/o/kernel-P-" << (filenameCounter++) << ".jpg";

    cv::imwrite(ss.str(), output_mat);

    p_mats->push_back(output_mat);
  }
  std::unordered_map<std::string, void *> outputData;

  outputData["p_mats"] = ((void *) p_mats);
  logger << "after bicubicRemapToSpherical" << std::endl;

  t = time(0);
  printf("[Kernel-P-end]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  return outputData;
}
