#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"

typedef int i32;
using namespace elixir;

std::unordered_map<std::string, void *> KernelI::execute (
  std::vector<elixir::Data *>& dataList) {
  assert(dataList.empty());

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

  cap.set(CV_CAP_PROP_POS_FRAMES, frameNumber_);

  std::vector<cv::Mat> *outputMats = new std::vector<cv::Mat>();

  for (int frameNum = 0; frameNum < batchSize_; ++frameNum) {
    //TODO ending: batchSize_ > frames left
    cv::Mat frame_col_mat;
    cap >> frame_col_mat;
    outputMats->push_back(frame_col_mat);
  }

  std::unordered_map<std::string, void *> outputData;
  outputData["frame_col_mats"] = ((void *) outputMats);

  return outputData;
}
