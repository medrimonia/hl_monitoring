#include "hl_monitoring/calibrated_image.h"

#include "hl_monitoring/utils.h"

#include <stdexcept>

namespace hl_monitoring
{

CalibratedImage::CalibratedImage(const cv::Mat & img,
                                 const Pose3D & pose,
                                 const IntrinsicParameters & camera_parameters) {
  cv::Size size;
  intrinsicToCV(camera_parameters, &camera_matrix, &camera_distortion_coeffs, &size);
  if (size.width != img.cols || size.height != img.rows) {
    throw std::logic_error("Img size doest not match size declared in camera_parameters");
  }
  pose3DToCV(pose, &rvec, &tvec);
}

}
