#include "surround360_kernels.h"

typedef int i32;

std::unordered_map<std::string, void *> KernelI::execute override (
  std::vector<elixir::Data>& dataList) {
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
