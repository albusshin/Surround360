#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "source/scanner_kernels/surround360.pb.h"

#include <opencv2/video.hpp>
#include <string>
typedef int i32;

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
    void execute(std::vector<cv::Mat>& frame_col_mats,
                 const int camIdx,
                 std::vector<cv::Mat>& output_mats) {
      std::cout << "execute, frame_col_mats.size() == " << frame_col_mats.size()
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
      i32 input_count = (i32)frame_col_mats.size();
      std::cout << "input_count == " << input_count << std::endl;
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

      for (i32 i = 0; i < input_count; ++i) {
        std::cout << "iteration i = " << i << std::endl;
        cv::Mat input = frame_col_mats[i];
        cv::Mat tmp;
        cv::cvtColor(input, tmp, CV_BGR2BGRA);
        std::cout << "after cvtColor()" << std::endl;

        // NOTE: cv::Mat (int rows, int cols, int type)
        cv::Mat projection_image(output_image_height, output_image_width, cv_madetype);

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
          projection_image, //dst
          tmp, //src
          rig_->rigSideOnly[camIdx_],
          leftAngle_,
          rightAngle_,
          topAngle_,
          bottomAngle_);
        std::cout << "after bicubicRemapToSpherical" << std::endl;

        output_mats.push_back(projection_image);

        /**
         * albusshin:
         *
         * inline void insert_frame(ElementList& column, Frame* frame) {
         *   column.push_back(::scanner::Element{frame});
         * }
         *
         */
      }
    }


    cv::Mat get_dummy_frame (int width, int height) {
      int channels = 4;
      int cv_type = CV_8U;
      int cv_madetype = CV_MAKETYPE(cv_type, channels);

      return cv::Mat(width, height, cv_madetype);
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

}

int main(int argc, char *argv[]) {
  surround360::proto::ProjectSphericalArgs args;
  args.set_eqr_width(8400);
  args.set_eqr_height(4096);
  args.set_camera_rig_path("/home/ubuntu/d/a/palace3/camera_rig.json");
  surround360::ProjectSphericalKernelCPUExtracted project_kernel(args);
  std::vector<cv::Mat> frame_col_mats;

  std::string filename= "/home/ubuntu/d/a/palace3/rgb/cam0/vid.mp4";

  cv::VideoCapture capture(filename);
  if (!capture.isOpened) {
    std::cerr << "ERROR opening file "
              << filename
              << " as cv::VideoCapture"
              << std::endl;
  }

  int channels = 4;
  int cv_type = CV_8U;
  int cv_madetype = CV_MAKETYPE(cv_type, channels);
  cv::Mat framemat(args.eqr_width(), args.eqr_height(), cv_madetype);
  capture >> framemat; // get new frame from video
  frame_col_mats.push_back(framemat);
  std::cout << "Made framemat framemat "
            << " args.eqr_width = " << args.eqr_width()
            << " args.eqr_height = " << args.eqr_height()
            << " framemat.rows (height) == " << framemat.rows
            << " framemat.cols (width) == " << framemat.cols
            << std::endl;


  std::vector<cv::Mat> output_mats;
  int camera_id = 0;
  std::cout << "Before execution of kernel" << std::endl;
  project_kernel.execute(frame_col_mats, camera_id, output_mats);
  std::cout << "Done!" << std::endl;
  std::cout << output_mats.size() << std::endl;
}
