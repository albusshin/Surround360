#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "optical_flow/NovelView.h"
#include "util/MathUtil.h"
#include "source/scanner_kernels/surround360.pb.h"
#include "optical_flow/NovelView.h"
#include "scanner/api/kernel.h"
#include "scanner/api/op.h"
#include "scanner/util/memory.h"
#include "scanner/util/opencv.h"

#include <opencv2/video.hpp>
#include <string>
#include <sstream>

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
      FrameInfo info(output_image_height, output_image_width, 4, FrameType::U8);
      std::vector<Frame*> frames = new_frames(device_, info, input_count * 2);

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

  // First step arg
  surround360::proto::ProjectSphericalArgs project_args;
  project_args.set_eqr_width(8400);
  project_args.set_eqr_height(4096);
  project_args.set_camera_rig_path(CAMERA_RIG_PATH);
  surround360::ProjectSphericalKernelCPUExtracted project_kernel(project_args);

  // Extract frame from cam0
  // NOTE Using 6 and 7 here, in case cam0 is for something special
  cv::Mat frame_col_mat0;
  std::string filename_0;
  get_video_filename(6, filename_0);
  getOneFrame(filename_0, frame_col_mat0);
  std::cout << "Made framemat framemat "
            << " project_args.eqr_width = " << project_args.eqr_width()
            << " project_args.eqr_height = " << project_args.eqr_height()
            << " frame_col_mat0.rows (height) == " << frame_col_mat0.rows
            << " frame_col_mat0.cols (width) == " << frame_col_mat0.cols
            << std::endl;

  // Extract frame from cam1
  cv::Mat frame_col_mat1;
  std::string filename_1;
  get_video_filename(7, filename_1);
  getOneFrame(filename_1, frame_col_mat1);

  std::cout << "Before execution of kernel" << std::endl;
  // Calculate left_project
  cv::Mat left_project;
  project_kernel.execute(frame_col_mat0, 0, left_project);
  std::cout << "Done left_project = "
            << left_project.cols
            << " * "
            << left_project.rows
            << " * "
            << left_project.channels()
            << std::endl;
  // Calculate right_project
  cv::Mat right_project;
  project_kernel.execute(frame_col_mat1, 1, right_project);
  std::cout << "Done right_project = "
            << right_project.cols
            << " * "
            << right_project.rows
            << " * "
            << right_project.channels()
            << std::endl;

  // Second step arg
  surround360::proto::TemporalOpticalFlowArgs temporal_args;

  temporal_args.set_camera_rig_path(CAMERA_RIG_PATH);
  temporal_args.set_flow_algo(FLOW_ALGO);

  surround360::TemporalOpticalFlowKernelCPUExtracted temporal_kernel(temporal_args);

  std::cout << "Before temporal_kernel.new_frame_info" << std::endl;
  temporal_kernel.new_frame_info(left_project.cols, left_project.rows);

  // TODO Counter-clockwise or clockwise?
  std::cout << "Before temporal_kernel.execute" << std::endl;
  cv::Mat left_flow;
  cv::Mat right_flow;

  temporal_kernel.execute(left_project, right_project, left_flow, right_flow);
  std::cout << "After temporal_kernel.execute" << std::endl;

  std::cout << "After temporal_kernel.execute" << std::endl;
  std::cout << "Done left_flow = "
            << left_flow.cols
            << " * "
            << left_flow.rows
            << " * "
            << left_flow.channels()
            << std::endl;


  // Third step arg
  surround360::proto::RenderStereoPanoramaChunkArgs render_args;
  render_args.set_camera_rig_path(CAMERA_RIG_PATH);
  render_args.set_flow_algo(FLOW_ALGO);
  render_args.set_eqr_width(8400);
  render_args.set_eqr_height(4096);
  render_args.set_zero_parallax_dist(10000);
  render_args.set_interpupilary_dist(6.4);

  surround360::RenderStereoPanoramaChunkKernelCPUExtracted render_kernel(render_args);
  render_kernel.new_frame_info(left_flow.cols, left_flow.rows);
  cv::Mat chunkL, chunkR;
  std::cout << "Before render_kernel.execute" << std::endl;
  render_kernel.execute(left_project, right_project, left_flow, right_flow, chunkL, chunkR);
  std::cout << "After render_kernel.execute" << std::endl;
  std::cout << "Done chunkL = "
            << chunkL.cols
            << " * "
            << chunkL.rows
            << " * "
            << chunkL.channels()
            << std::endl;

}
