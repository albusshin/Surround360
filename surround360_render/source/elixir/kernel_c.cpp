#include <string>
#include "surround360_kernels.h"
#include "pthread.h"
#include "nullbuf.h"
#include "worker.h"
#include <thread>
#include <mutex>

typedef int i32;
using namespace elixir;

int counter = 0;
std::mutex mtx;

void KernelC::new_frame_info(
  int camImageWidth, int camImageHeight) {
  camImageWidth_ = camImageWidth;
  camImageHeight_ = camImageHeight;
  logger << "[Concat]\t"
         << "new_frame_info: "
         << "camImageWidth = "
         << camImageWidth
         << "camImageHeight = "
         << camImageHeight
         << std::endl;
  const int numCams = 14;
  const float cameraRingRadius = rig_->getRingRadius();
  const float camFovHorizontalDegrees =
    2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
  const float fovHorizontalRadians = toRadians(camFovHorizontalDegrees);
  const float overlapAngleDegrees =
    (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
  const int overlapImageWidth =
    float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);

  const float v =
    atanf(zero_parallax_dist_ / (interpupilary_dist_ / 2.0f));
  const float psi =
    asinf(sinf(v) * (interpupilary_dist_ / 2.0f) / cameraRingRadius);
  const float vergeAtInfinitySlabDisplacement =
    psi * (float(camImageWidth) / fovHorizontalRadians);
  const float theta = -M_PI / 2.0f + v + psi;
  zeroParallaxNovelViewShiftPixels_ =
    float(eqr_width_) * (theta / (2.0f * M_PI));
  if (!left_) {
    zeroParallaxNovelViewShiftPixels_ *= -1;
  }
}

/*
  Accept:
  [0]: chunkL / chunkR
  ...
  [13]: chunkL / chunkR

  Output:
  pano
*/
std::unordered_map<std::string, void *> KernelC::execute (
  std::vector<elixir::Data *>& dataList) {

  logger << "[KernelC T"
         << elixir::Worker::getWorkerId()
         << "]\t"
         << "execute()"
         << endl;

  time_t t = time(0);
  printf("[Kernel-C-start]: %ld: %d\n", t, elixir::Worker::getWorkerId());

  string chunkKey;
  if (left_) {
    chunkKey = "chunkLs";
  } else {
    chunkKey = "chunkRs";
  }

  i32 num_chunks = dataList.size();
  assert(num_chunks == 14);

  int batchSize = ((vector<cv::Mat> *) dataList[0]->data[chunkKey])->size();
  vector<cv::Mat> *panos = new vector<cv::Mat>();

  for (int frameNum = 0; frameNum < batchSize; ++frameNum) {

    cv::Mat& tmp_input_chunk =
      (*(vector<cv::Mat> *) dataList[0]->data[chunkKey])[frameNum];

    int camImageWidth = tmp_input_chunk.cols;
    int camImageHeight = tmp_input_chunk.rows;

    new_frame_info(camImageWidth, camImageHeight);

    size_t output_image_width = camImageWidth_ * num_chunks;
    size_t output_image_height = camImageHeight_;
    output_image_width += (output_image_width % 2);
    output_image_height += (output_image_height % 2);
    size_t output_image_size =
      output_image_width * output_image_height * 3;

    std::vector<cv::Mat> pano_chunks(num_chunks, Mat());
    for (i32 c = 0; c < num_chunks; ++c) {
      vector<cv::Mat>& input_chunks = (*(vector<cv::Mat> *) (dataList[c]->data[chunkKey]));
      cv::Mat& input_chunk = input_chunks[frameNum];
      cv::cvtColor(input_chunk, pano_chunks[c], CV_BGRA2BGR);
    }
    cv::Mat pano;

    pano = stackHorizontal(pano_chunks);
    pano = offsetHorizontalWrap(pano, zeroParallaxNovelViewShiftPixels_);

    panos->push_back(pano);

    if (left_) {
      stringstream ss;
      ss <<  "/home/ubuntu/o/panoL-elixir" << counter << ".jpg";
      string name = ss.str();
      ss.clear();
      fprintf(stdout, "[c-kernel] name: %s\n", name.c_str());
      cv::imwrite(name, pano);
    } else {
      stringstream ss;
      ss <<  "/home/ubuntu/o/panoR-elixir" << counter << ".jpg";
      string name = ss.str();
      ss.clear();
      fprintf(stdout, "[c-kernel] name: %s\n", name.c_str());
      cv::imwrite(name, pano);
    }

    fprintf(stdout, "[c-kernel] finish save\n");
    fprintf(stdout, "[c-kernel] counter: %d\n", counter);
    if (!left_) {
      mtx.lock();
      counter += 1;
      mtx.unlock();
    }
    fprintf(stdout, "[c-kernel] counter: %d\n", counter);

    t = time(0);
    printf("[Kernel-C-end]: %ld: %d\n", t, elixir::Worker::getWorkerId());
  }
  std::unordered_map<std::string, void *> outputData;
  outputData["panos"] = ((void *) panos);

  return outputData;
}
