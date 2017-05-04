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

typedef int i32;


// c2.xlarge
// 52.14.253.28
// r4.8xlarge
// 52.15.64.158

/*
  NOTE
  python scripts/scanner_process_video.py
  --flow_alg pixflow_search_20
  --rig_json_file ~/d/a/palace3/camera_rig.json
  --quality 8k
  --start_frame 0
  --end_frame 2
  --surround360_render_dir .
  --root_dir ~/d/a/palace3
  --verbose


  if quality == "3k":
  render_params["SHARPENNING"]                  = 0.25
  render_params["EQR_WIDTH"]                    = 3080
  render_params["EQR_HEIGHT"]                   = 1540
  render_params["FINAL_EQR_WIDTH"]              = 3080
  render_params["FINAL_EQR_HEIGHT"]             = 3080
  elif quality == "4k":
  render_params["SHARPENNING"]                  = 0.25
  render_params["EQR_WIDTH"]                    = 4200
  render_params["EQR_HEIGHT"]                   = 1024
  render_params["FINAL_EQR_WIDTH"]              = 4096
  render_params["FINAL_EQR_HEIGHT"]             = 2048
  elif quality == "6k":
  render_params["SHARPENNING"]                  = 0.25
  render_params["EQR_WIDTH"]                    = 6300
  render_params["EQR_HEIGHT"]                   = 3072
  render_params["FINAL_EQR_WIDTH"]              = 6144
  render_params["FINAL_EQR_HEIGHT"]             = 6144
  elif quality == "8k":
  render_params["SHARPENNING"]                  = 0.25
  render_params["EQR_WIDTH"]                    = 8400
  render_params["EQR_HEIGHT"]                   = 4096
  render_params["FINAL_EQR_WIDTH"]              = 8192
  render_params["FINAL_EQR_HEIGHT"]             = 8192

*/

namespace surround360 {

  using namespace optical_flow;
  using namespace math_util;

  class ProjectSphericalKernelCPUExtracted {
  public:
    ProjectSphericalKernelCPUExtracted(const surround360::proto::ProjectSphericalArgs &args) {
      args_ = args;
      // Initialize camera rig
      rig_.reset(new RigDescription(args_.camera_rig_path()));

      hRadians_ = 2 * approximateFov(rig_->rigSideOnly, false);
      vRadians_ = 2 * approximateFov(rig_->rigSideOnly, true);
    }

    void reset() {
      is_reset_ = true;
    }

    /**
       NOTE :reference
       * using ElementList = std::vector<Element>;
       * using BatchedColumns = std::vector<ElementList>;
       *
       * So BatchedColumns is vector<vector<Element>>
       */
    void execute(cv::Mat& frame_col_mat,
                 const int camIdx,
                 cv::Mat& output_mat) {
      std::cout << "execute "
                << ", camIdx == " << camIdx << std::endl;

      if (is_reset_) {
        std::cout << "is_reset_ == true" << std::endl;
        // Use the new camera id to update the spherical projection parameters
        is_reset_ = false;

        camIdx_ = camIdx;
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
        NOTE :reference
        cv::Mat frame_to_mat(Frame* frame) {
        return cv::Mat(frame->height(), frame->width(),
        frame_to_cv_type(frame->type, frame->channels()), frame->data);
        }

        NOTE :reference
        int frame_to_cv_type(FrameType type, int channels) {
        int cv_type;
        switch (type) {
        case FrameType::U8: {
        cv_type = CV_8U;
        break;
        }
        case FrameType::F32: {
        cv_type = CV_32F;
        break;
        }
        case FrameType::F64: {
        cv_type = CV_64F;
        break;
        }
        }
        return CV_MAKETYPE(cv_type, channels);
        }
      */
      size_t output_image_width = args_.eqr_width() * hRadians_ / (2 * M_PI);
      size_t output_image_height = args_.eqr_height() * vRadians_ / M_PI;
      size_t output_image_size = output_image_width * output_image_height * 4;
      std::cout << "output_image_width == " << output_image_width
                << ", output_image_height == " << output_image_height
                << ", output_image_size == " << output_image_size
                << std::endl;



      /* NOTE :reference
         int FrameInfo::channels() const { return shape[2]; }
      */
      int channels = 4;
      int cv_type = CV_8U;
      int cv_madetype = CV_MAKETYPE(cv_type, channels);


      std::cout << "P: input = "
                << frame_col_mat.cols
                << " * "
                << frame_col_mat.rows
                << " * "
                << frame_col_mat.channels()
                << std::endl;


      cv::Mat tmp;
      cv::cvtColor(frame_col_mat, tmp, CV_BGR2BGRA);
      std::cout << "after cvtColor()" << std::endl;

      // NOTE: cv::Mat (int rows, int cols, int type)
      output_mat = cv::Mat(output_image_height, output_image_width, cv_madetype);

      /**
         NOTE
         void bicubicRemapToSpherical(
         Mat& dst,
         const Mat& src,
         const Camera& camera,
         const float leftAngle,
         const float rightAngle,
         const float topAngle,
         const float bottomAngle);
      */
      surround360::warper::bicubicRemapToSpherical(
        output_mat, //dst
        tmp, //src
        rig_->rigSideOnly[camIdx_],
        leftAngle_,
        rightAngle_,
        topAngle_,
        bottomAngle_);
      std::cout << "after bicubicRemapToSpherical" << std::endl;

      /**
       * albusshin:
       *
       * inline void insert_frame(ElementList& column, Frame* frame) {
       *   column.push_back(::scanner::Element{frame});
       * }
       *
       */
    }

  private:
    surround360::proto::ProjectSphericalArgs args_;
    std::unique_ptr<RigDescription> rig_;
    bool is_reset_ = true;

    float hRadians_;
    float vRadians_;

    int camIdx_;
    float leftAngle_;
    float rightAngle_;
    float topAngle_;
    float bottomAngle_;
  };

  class TemporalOpticalFlowKernelCPUExtracted {
  public:
    TemporalOpticalFlowKernelCPUExtracted(
      const surround360::proto::TemporalOpticalFlowArgs &args) {

      args_ = args;

      rig_.reset(new RigDescription(args_.camera_rig_path()));

      overlap_image_width_ = -1;
      novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(args_.flow_algo()));

      reset();
    }

    void reset() {
      prev_frame_flow_l_to_r_ = cv::Mat();
      prev_frame_flow_r_to_l_ = cv::Mat();
      prev_overlap_image_l_ = cv::Mat();
      prev_overlap_image_r_ = cv::Mat();
    }

    // TODO: must this function be invoked before adding a new frame?
    void new_frame_info(int camImageWidth, int camImageHeight) {
      camImageHeight_ = camImageHeight;
      const int numCams = 14;
      const float cameraRingRadius = rig_->getRingRadius();
      const float camFovHorizontalDegrees =
        2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
      const float fovHorizontalRadians = toRadians(camFovHorizontalDegrees);
      const float overlapAngleDegrees =
        (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
      overlap_image_width_ =
        float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);
    }

    void execute(cv::Mat& left_input,
                 cv::Mat& right_input,
                 cv::Mat& left_flow,
                 cv::Mat& right_flow) {
      assert(overlap_image_width_ != -1);

      size_t output_image_width = overlap_image_width_;
      size_t output_image_height = camImageHeight_;

      std::cout << "T: left_input = "
                << left_input.cols
                << " * "
                << left_input.rows
                << " * "
                << left_input.channels()
                << std::endl;

      cv::Mat left_overlap_input =
        left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                            overlap_image_width_, left_input.rows));
      cv::Mat right_overlap_input =
        right_input(cv::Rect(0, 0,
                             overlap_image_width_, right_input.rows));

      std::cout << "T: left_overlap_input = "
                << left_overlap_input.cols
                << " * "
                << left_overlap_input.rows
                << " * "
                << left_overlap_input.channels()
                << std::endl;

      novel_view_gen_->prepare(left_overlap_input, right_overlap_input,
                               prev_frame_flow_l_to_r_, prev_frame_flow_r_to_l_,
                               prev_overlap_image_l_, prev_overlap_image_r_);

      //  NOTE txin: This is to copy the state for next frame.
      left_overlap_input.copyTo(prev_overlap_image_l_);
      right_overlap_input.copyTo(prev_overlap_image_r_);
      prev_frame_flow_l_to_r_ = novel_view_gen_->getFlowLtoR();
      prev_frame_flow_r_to_l_ = novel_view_gen_->getFlowRtoL();

      left_flow = novel_view_gen_->getFlowLtoR();
      right_flow = novel_view_gen_->getFlowRtoL();
    }

  private:
    surround360::proto::TemporalOpticalFlowArgs args_;
    std::unique_ptr<RigDescription> rig_;
    int overlap_image_width_;
    int camImageHeight_;

    std::unique_ptr<NovelViewGenerator> novel_view_gen_;
    cv::Mat prev_frame_flow_l_to_r_;
    cv::Mat prev_frame_flow_r_to_l_;
    cv::Mat prev_overlap_image_l_;
    cv::Mat prev_overlap_image_r_;
  };

  class RenderStereoPanoramaChunkKernelCPUExtracted {
  public:
    RenderStereoPanoramaChunkKernelCPUExtracted(surround360::proto::RenderStereoPanoramaChunkArgs args) {
      args_ = args;

      rig_.reset(new RigDescription(args_.camera_rig_path()));

      overlap_image_width_ = -1;
      novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(args_.flow_algo()));
    }

    void new_frame_info(int camImageWidth, int camImageHeight) {
      camImageHeight_ = camImageHeight;

      const int numCams = 14;
      const float cameraRingRadius = rig_->getRingRadius();
      const float camFovHorizontalDegrees =
        2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
      fov_horizontal_radians_ = toRadians(camFovHorizontalDegrees);
      const float overlapAngleDegrees =
        (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
      overlap_image_width_ = float(camImageWidth) *
        (overlapAngleDegrees / camFovHorizontalDegrees);
      num_novel_views_ =
        camImageWidth - overlap_image_width_; // per image pair

      const float v =
        atanf(args_.zero_parallax_dist() / (args_.interpupilary_dist() / 2.0f));
      const float psi =
        asinf(sinf(v) * (args_.interpupilary_dist() / 2.0f) / cameraRingRadius);
      const float vergeAtInfinitySlabDisplacement =
        psi * (float(camImageWidth) / fov_horizontal_radians_);
      const float theta = -M_PI / 2.0f + v + psi;
      const float zeroParallaxNovelViewShiftPixels =
        float(args_.eqr_width()) * (theta / (2.0f * M_PI));

      int currChunkX = 0;
      lazy_view_buffer_.reset(new LazyNovelViewBuffer(args_.eqr_width() / numCams,
                                                      camImageHeight));
      for (int nvIdx = 0; nvIdx < num_novel_views_; ++nvIdx) {
        const float shift = float(nvIdx) / float(num_novel_views_);
        const float slabShift =
          float(camImageWidth) * 0.5f - float(num_novel_views_ - nvIdx);

        for (int v = 0; v < camImageHeight; ++v) {
          lazy_view_buffer_->warpL[currChunkX][v] =
            Point3f(slabShift + vergeAtInfinitySlabDisplacement, v, shift);
          lazy_view_buffer_->warpR[currChunkX][v] =
            Point3f(slabShift - vergeAtInfinitySlabDisplacement, v, shift);
        }
        ++currChunkX;
      }
    }

    void execute(cv::Mat& left_input,
                 cv::Mat& right_input,
                 cv::Mat& left_flow,
                 cv::Mat& right_flow,
                 cv::Mat& chunkL,
                 cv::Mat& chunkR) {
      assert(overlap_image_width_ != -1);

      size_t output_image_width = num_novel_views_;
      size_t output_image_height = camImageHeight_;
      size_t output_image_size =
        output_image_width * output_image_height * 4;

      cv::Mat left_overlap_input =
        left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                            overlap_image_width_, left_input.rows));
      cv::Mat right_overlap_input =
        right_input(cv::Rect(0, 0,
                             overlap_image_width_, right_input.rows));


      // Initialize NovelViewGenerator with images and flow since we are
      // bypassing the prepare method
      novel_view_gen_->setImageL(left_overlap_input);
      novel_view_gen_->setImageR(right_overlap_input);
      novel_view_gen_->setFlowLtoR(left_flow);
      novel_view_gen_->setFlowRtoL(right_flow);
      std::pair<Mat, Mat> lazyNovelChunksLR =
        novel_view_gen_->combineLazyNovelViews(*lazy_view_buffer_.get());
      chunkL = lazyNovelChunksLR.first;
      chunkR = lazyNovelChunksLR.second;
    }

  private:
    int camImageHeight_;
    surround360::proto::RenderStereoPanoramaChunkArgs args_;
    std::unique_ptr<RigDescription> rig_;
    float fov_horizontal_radians_;
    int overlap_image_width_;
    int num_novel_views_;

    std::unique_ptr<NovelViewGenerator> novel_view_gen_;
    std::unique_ptr<LazyNovelViewBuffer> lazy_view_buffer_;
  };

  class ConcatPanoramaChunksKernelCPUExtracted {
  public:
    ConcatPanoramaChunksKernelCPUExtracted(surround360::proto::ConcatPanoramaChunksArgs args) {

      args_ = args;

      rig_.reset(new RigDescription(args_.camera_rig_path()));
    }

    void new_frame_info(int camImageWidth, int camImageHeight) {
      camImageWidth_ = camImageWidth;
      camImageHeight_ = camImageHeight;
      std::cout << "[Concat]\t"
                << "new_frame_info: "
                << "camImageWidth = "
                << camImageWidth
                << "camImageHeight = "
                << camImageHeight;
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
        atanf(args_.zero_parallax_dist() / (args_.interpupilary_dist() / 2.0f));
      const float psi =
        asinf(sinf(v) * (args_.interpupilary_dist() / 2.0f) / cameraRingRadius);
      const float vergeAtInfinitySlabDisplacement =
        psi * (float(camImageWidth) / fovHorizontalRadians);
      const float theta = -M_PI / 2.0f + v + psi;
      zeroParallaxNovelViewShiftPixels_ =
        float(args_.eqr_width()) * (theta / (2.0f * M_PI));
      if (!args_.left()) {
        zeroParallaxNovelViewShiftPixels_ *= -1;
      }
    }

    void execute(std::vector<cv::Mat>& input_chunks,
                 cv::Mat& pano) {

      i32 num_chunks = input_chunks.size();
      size_t output_image_width = camImageWidth_ * num_chunks;
      size_t output_image_height = camImageHeight_;
      output_image_width += (output_image_width % 2);
      output_image_height += (output_image_height % 2);
      size_t output_image_size =
        output_image_width * output_image_height * 3;

      std::vector<cv::Mat> pano_chunks(num_chunks, Mat());
      for (i32 c = 0; c < num_chunks; ++c) {
        cv::cvtColor(input_chunks[c],
                     pano_chunks[c], CV_BGRA2BGR);
      }
      pano = stackHorizontal(pano_chunks);
      pano = offsetHorizontalWrap(pano, zeroParallaxNovelViewShiftPixels_);
    }

  private:
    surround360::proto::ConcatPanoramaChunksArgs args_;
    std::unique_ptr<RigDescription> rig_;
    float zeroParallaxNovelViewShiftPixels_;
    int camImageWidth_;
    int camImageHeight_;
  };

}

void get_video_filename(int camId, std::string& outstr) {
  assert(camId >= 0 && camId <= 16);
  std::stringstream ss;
  ss << "/home/ubuntu/d/a/palace3/rgb/cam" << camId << "/vid.mp4";
  ss >> outstr;
}

const std::string CAMERA_RIG_PATH = "/home/ubuntu/d/a/palace3/camera_rig.json";
const std::string FLOW_ALGO = "pixflow_search_20";

void getOneFrame(const std::string& filename,
                 cv::Mat &output_mat) {
  cv::VideoCapture capture(filename);
  if (!capture.isOpened()) {
    std::cerr << "ERROR opening file "
              << filename
              << " as cv::VideoCapture"
              << std::endl;
    assert(false);
  }

  capture >> output_mat;
}

int main(int argc, char *argv[]) {

  const int numCamera = 14;

  time_t start = time(0);
  std::cout << "[Main]\t"
            << "start time: "
            << start
            << std::endl;

  // First step arg
  surround360::proto::ProjectSphericalArgs project_args;
  project_args.set_eqr_width(8400);
  project_args.set_eqr_height(4096);
  project_args.set_camera_rig_path(CAMERA_RIG_PATH);
  std::vector<surround360::ProjectSphericalKernelCPUExtracted *> project_kernels;

  std::vector<cv::Mat> frame_col_mats(numCamera, cv::Mat());
  std::vector<std::string> filenames(numCamera);

  std::vector<cv::Mat> projects(numCamera, cv::Mat());

  int offset = 1;
  for (int i = 0; i < numCamera; ++i) {
    int cameraId = i + offset;
    surround360::ProjectSphericalKernelCPUExtracted *project_kernel =
      new surround360::ProjectSphericalKernelCPUExtracted(project_args);
    project_kernels.push_back(project_kernel);
    get_video_filename(cameraId, filenames[i]);

    std::cout << "[Main]\t"
              << "cameraId == "
              << cameraId
              << ", filenames["
              << i
              << "] = "
              << filenames[i]
              << std::endl;

    getOneFrame(filenames[i], frame_col_mats[i]);
    std::cout << "[Main]\t"
              << "Made frame_col_mat["
              << i
              << "] = "
              << frame_col_mats[i].rows
              << " * "
              << frame_col_mats[i].cols
              << " * "
              << frame_col_mats[i].channels()
              << std::endl;

    std::cout << "Before execution of kernel" << std::endl;
    // Calculate right project
    project_kernels[i]->execute(frame_col_mats[i], i, projects[i]);
    std::cout << "[Main]\t"
              << "Done project["
              << i
              << "] = "
              << projects[i].cols
              << " * "
              << projects[i].rows
              << " * "
              << projects[i].channels()
              << std::endl;

    cv::Mat towrite;
    cv::cvtColor(projects[i], towrite, CV_BGRA2BGR);

    std::stringstream ss;
    ss << "/home/ubuntu/o/projects_elixir_" << i << ".jpg";
    cv::imwrite(ss.str(), towrite);
  }
  time_t after_step1 = time(0);
  std::cout << "[Main]\t"
            << "after_step1 time: "
            << after_step1
            << ", step1 = "
            << (after_step1 - start)
            << std::endl;

  // Second step arg
  surround360::proto::TemporalOpticalFlowArgs temporal_args;

  temporal_args.set_camera_rig_path(CAMERA_RIG_PATH);
  temporal_args.set_flow_algo(FLOW_ALGO);

  std::vector<surround360::TemporalOpticalFlowKernelCPUExtracted *> temporal_kernels;

  std::vector<cv::Mat> left_flows(numCamera, cv::Mat());
  std::vector<cv::Mat> right_flows(numCamera, cv::Mat());

  for (int i = 0; i < numCamera; ++i) {
    surround360::TemporalOpticalFlowKernelCPUExtracted *temporal_kernel =
      new surround360::TemporalOpticalFlowKernelCPUExtracted(temporal_args);
    temporal_kernels.push_back(temporal_kernel);
    cv::Mat& left_project = projects[i];
    cv::Mat& right_project = projects[(i + 1) % numCamera];

    std::cout << "[Main]\t"
              << "Before temporal_kernel.new_frame_info"
              <<"["
              << i
              << "]"
              << std::endl;
    temporal_kernels[i]->new_frame_info(left_project.cols, left_project.rows);

    std::cout << "[Main]\t"
              << "Before temporal_kernel.execute"
              <<"["
              << i
              << "]"
              << std::endl;

    temporal_kernels[i]->execute(left_project, right_project, left_flows[i], right_flows[i]);

    std::cout << "[Main]\t"
              << "Done left_flows"
              <<"["
              << i
              << "] = "
              << left_flows[i].cols
              << " * "
              << left_flows[i].rows
              << " * "
              << left_flows[i].channels()
              << std::endl;

    cv::Mat towrite_left;
    cv::cvtColor(left_flows[i], towrite_left, CV_BGRA2BGR);

    cv::Mat towrite_right;
    cv::cvtColor(right_flows[i], towrite_right, CV_BGRA2BGR);

    std::stringstream ss;
    ss << "/home/ubuntu/o/left_flow_elixir_" << i << ".jpg";
    cv::imwrite(ss.str(), towrite_left);
    ss.clear();
    ss << "/home/ubuntu/o/right_flow_elixir_" << i << ".jpg";
    cv::imwrite(ss.str(), towrite_right);
  }

  time_t after_step2 = time(0);
  std::cout << "[Main]\t"
            << "after_step2 time: "
            << after_step2
            << ", step2 = "
            << (after_step2 - after_step1)
            << std::endl;


  // Third step arg
  surround360::proto::RenderStereoPanoramaChunkArgs render_args;
  render_args.set_camera_rig_path(CAMERA_RIG_PATH);
  render_args.set_flow_algo(FLOW_ALGO);
  render_args.set_eqr_width(8400);
  render_args.set_eqr_height(4096);
  render_args.set_zero_parallax_dist(10000);
  render_args.set_interpupilary_dist(6.4);

  std::vector<surround360::RenderStereoPanoramaChunkKernelCPUExtracted *>
    render_kernels;

  std::vector<cv::Mat> chunkLs(numCamera, cv::Mat());
  std::vector<cv::Mat> chunkRs(numCamera, cv::Mat());

  for (int i = 0; i < numCamera; ++i) {
    surround360::RenderStereoPanoramaChunkKernelCPUExtracted *render_kernel = new
      surround360::RenderStereoPanoramaChunkKernelCPUExtracted(render_args);
    render_kernels.push_back(render_kernel);
    std::cout << "[Main]\t"
              << "Before render_kernel.new_frame_info"
              <<"["
              << i
              << "]"
              << std::endl;

    render_kernels[i]->new_frame_info(left_flows[i].cols, left_flows[i].rows);


    std::cout << "[Main]\t"
              << "Before render_kernel.execute"
              <<"["
              << i
              << "]"
              << std::endl;

    render_kernels[i]->execute(projects[i],
                               projects[(i + 1) % numCamera],
                               left_flows[i],
                               right_flows[i],
                               chunkLs[i],
                               chunkRs[i]);

    std::cout << "[Main]\t"
              << "After render_kernel.execute"
              <<"["
              << i
              << "]"
              << std::endl;


    std::cout << "[Main]\t"
              << "Done chunkLs"
              <<"["
              << i
              << "] = "
              << chunkLs[i].cols
              << " * "
              << chunkLs[i].rows
              << " * "
              << chunkLs[i].channels()
              << std::endl;

    cv::Mat towrite_left;
    cv::cvtColor(chunkLs[i], towrite_left, CV_BGRA2BGR);

    cv::Mat towrite_right;
    cv::cvtColor(chunkRs[i], towrite_right, CV_BGRA2BGR);

    std::stringstream ss;
    ss << "/home/ubuntu/o/chunkL_elixir_" << i << ".jpg";
    cv::imwrite(ss.str(), towrite_left);
    ss.clear();
    ss << "/home/ubuntu/o/chunkR_elixir" << i << ".jpg";
    cv::imwrite(ss.str(), towrite_right);

  }

  time_t after_step3 = time(0);
  std::cout << "[Main]\t"
            << "after_step3 time: "
            << after_step3
            << ", step3 = "
            << (after_step3 - after_step2)
            << std::endl;

  surround360::proto::ConcatPanoramaChunksArgs concat_args_left;
  concat_args_left.set_camera_rig_path(CAMERA_RIG_PATH);
  concat_args_left.set_eqr_width(8400);
  concat_args_left.set_eqr_height(4096);
  concat_args_left.set_zero_parallax_dist(10000);
  concat_args_left.set_interpupilary_dist(6.4);
  concat_args_left.set_left(true);
  
  surround360::proto::ConcatPanoramaChunksArgs concat_args_right;
  concat_args_right.set_camera_rig_path(CAMERA_RIG_PATH);
  concat_args_right.set_eqr_width(8400);
  concat_args_right.set_eqr_height(4096);
  concat_args_right.set_zero_parallax_dist(10000);
  concat_args_right.set_interpupilary_dist(6.4);
  concat_args_right.set_left(false);

  surround360::ConcatPanoramaChunksKernelCPUExtracted concat_kernel_left(concat_args_left);
  surround360::ConcatPanoramaChunksKernelCPUExtracted concat_kernel_right(concat_args_right);

  concat_kernel_left.new_frame_info(chunkLs[0].cols, chunkLs[0].rows);
  concat_kernel_right.new_frame_info(chunkRs[0].cols, chunkRs[0].rows);

  std::cout << "[Main]\t"
            << "Before concat_kernel_left.execute"
            << std::endl;

  cv::Mat panoL, panoR;
  concat_kernel_left.execute(chunkLs, panoL);
  std::cout << "[Main]\t"
            << "Done panoL = "
            << panoL.cols
            << " * "
            << panoL.rows
            << " * "
            << panoL.channels()
            << std::endl;

  std::cout << "[Main]\t"
            << "Before concat_kernel_right.execute"
            << std::endl;
  concat_kernel_right.execute(chunkRs, panoR);
  std::cout << "[Main]\t"
            << "Done panoR = "
            << panoR.cols
            << " * "
            << panoR.rows
            << " * "
            << panoR.channels()
            << std::endl;

  time_t end = time(0);
  std::cout << "[Main]\t"
            << "end time: "
            << end 
            << ", step4 = "
            << (end - after_step3)
            << ", end - start = "
            << (end - start)
            << std::endl;

  cv::imwrite( "/home/ubuntu/o/panoL.jpg", panoL);
  cv::imwrite( "/home/ubuntu/o/panoR.jpg", panoR);
}
