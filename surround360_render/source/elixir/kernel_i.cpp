#include "surround360_kernels.h"

typedef int i32;

std::unordered_map<std::string, void *> KernelI::execute(
  std::vector<elixir::Data> dataList) {
  assert(dataList.empty());

  cv::VideoCapture cap(videoFilename);
  if(!cap.isOpened()){
    std::cout << "I: Cannot open video file "
              << videoFilename
              << std::endl;
    assert(false);
  }

  cap.set(CV_CAP_PROP_POS_FRAMES, frameNumber);

  cv::Mat *frame_col_mat = new cv::Mat();
  bool success = cap.read(frame_col_mat);
  if (!success){
    std::cout << "I: Cannot read frame_col_mat "
              << frameNumber
              << "from video file "
              << videoFilename
              << std::endl;
  }

  std::unordered_map<std::string, void *> outputData;
  outputData["frame_col_mat"]((void *) frame_col_mat);

  return outputData;
}
