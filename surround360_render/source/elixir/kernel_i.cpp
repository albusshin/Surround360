#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"

#include <sstream>

typedef int i32;
using namespace elixir;

static std::atomic<int> filenameCounter = 0;

std::unordered_map<std::string, void *> KernelI::execute (
  std::vector<elixir::Data *>& dataList) {
  assert(dataList.empty());

  time_t t = time(0);
  printf("[Kernel-I-start]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  logger << "[KernelI T"
         << elixir::Worker::getWorkerId()
         << "]\t"
         << "execute()"
         << endl;

  cv::VideoCapture cap(videoFilename_);
  if(!cap.isOpened()){
    std::cout << "I: Cannot open video file "
              << videoFilename_
              << std::endl;
    assert(false);
  }

  cap.set(CV_CAP_PROP_POS_FRAMES, startFrameIndex_);

  std::vector<cv::Mat> *outputMats = new std::vector<cv::Mat>();

  for (int frameNum = 0; frameNum < batchSize_; ++frameNum) {
    //TODO ending: batchSize_ > frames left
    cv::Mat frame_col_mat;
    cap >> frame_col_mat;
    outputMats->push_back(frame_col_mat);

    std::stringstream ss;
    ss <<  "/home/ubuntu/o/kernel-I-" << (filenameCounter++) << ".jpg";

    cv::imwrite(ss.str(), frame_col_mat);
  }

  std::unordered_map<std::string, void *> outputData;
  outputData["frame_col_mats"] = ((void *) outputMats);

  t = time(0);
  printf("[Kernel-I-end]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  return outputData;
}
