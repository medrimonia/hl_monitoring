#pragma once

#include "hl_monitoring/camera.pb.h"

#include <opencv2/core.hpp>

namespace hl_monitoring
{

/**
 * Represents an image along with its intrinsic and extrinsic parameters
 */
class CalibratedImage {
public:

  CalibratedImage(const cv::Mat & img,
                  const Pose3D & pose,
                  const IntrinsicParameters & camera_parameters);

private:
  cv::Mat img;
  cv::Mat camera_matrix;
  cv::Mat camera_distortion_coeffs;

  /**
   * Rotation vector (see cv::Rodrigues)
   */
  cv::Mat rvec;

  /**
   * Translation vector toward cameras optical center in the field referential
   */
  cv::Mat tvec;
};

}
