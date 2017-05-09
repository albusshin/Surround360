#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"

typedef int i32;
using namespace elixir;

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

  std::vector<cv::Mat> *outputMats = new std::vector<cv::Mat>();

  for (int frameNum = 0; frameNum < batchSize_; ++frameNum) {
    //TODO ending: batchSize_ > frames left
    cv::Mat frame_col_mat;
    (*cap_) >> frame_col_mat;
    outputMats->push_back(frame_col_mat);
  }

  std::unordered_map<std::string, void *> outputData;
  outputData["frame_col_mats"] = ((void *) outputMats);

  t = time(0);
  printf("[Kernel-I-end]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  return outputData;
}
