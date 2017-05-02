#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "source/scanner_kernels/surround360.pb.h"

#include <opencv2/video.hpp>
#include <string>
#include <sstream>
#include <tuple>

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

}

std::string& get_video_filename(int camId) {
  assert(camId >= 0 && camId <= 16);
  std::stringstream ss;
  ss << "/home/ubuntu/d/a/palace3/rgb/cam" << camId << "/vid.mp4";
  return ss.str();
}

const std::string CAMERA_RIG_PATH = "/home/ubuntu/d/a/palace3/camera_rig.json";
const std::string FLOW_ALGO = "pixflow_search_20";

void getOneFrame(std::string& filename,
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
  cv::Mat frame_col_mat0 = getOneFrame(get_video_filename(6));
  std::cout << "Made framemat framemat "
            << " project_args.eqr_width = " << project_args.eqr_width()
            << " project_args.eqr_height = " << project_args.eqr_height()
            << " frame_col_mat0.rows (height) == " << frame_col_mat0.rows
            << " frame_col_mat0.cols (width) == " << frame_col_mat0.cols
            << std::endl;

  // Extract frame from cam1
  cv::Mat frame_col_mat1 = getOneFrame(get_video_filename(7));

  std::cout << "Before execution of kernel" << std::endl;
  // Calculate output_mat0
  cv::Mat output_mat0;
  project_kernel.execute(frame_col_mat0, 0, output_mat0);
  std::cout << "Done output_mat0, width = "
            << output_mat0.cols
            << " height = "
            << output_mat0.rows
            << std::endl;
  // Calculate output_mat1
  cv::Mat output_mat1;
  project_kernel.execute(frame_col_mat1, 1, output_mat1);
  std::cout << "Done output_mat1, width = "
            << output_mat1.cols
            << " height = "
            << output_mat1.rows
            << std::endl;

  // Second step arg
  surround360::proto::TemporalOpticalFlowArgs temporal_args;

  temporal_args.set_camera_rig_path(CAMERA_RIG_PATH);
  temporal_args.set_flow_algo(FLOW_ALGO);

  surround360::TemporalOpticalFlowKernelCPUExtracted temporal_kernel(temporal_args);

  std::cout << "Before temporal_kernel.new_frame_info" << std::endl;
  temporal_kernel.new_frame_info(frame_col_mat0.cols, fram_col_mat0.rows);

  // TODO Counter-clockwise or clockwise?
  std::cout << "Before temporal_kernel.execute" << std::endl;
  cv::Mat left_flow;
  cv::Mat right_flow;

  temporal_kernel.execute(frame_col_mat0, frame_col_mat1, left_flow, right_flow);
  std::cout << "After temporal_kernel.execute" << std::endl;

  std::cout << "After temporal_kernel.execute" << std::endl;
  std::cout << "Done left_flow, width = "
            << left_flow.cols
            << " height = "
            << left_flow.rows
            << std::endl;

  std::cout << "Done right_flow, width = "
            << right_flow.cols
            << " height = "
            << right_flow.rows
            << std::endl;

}
