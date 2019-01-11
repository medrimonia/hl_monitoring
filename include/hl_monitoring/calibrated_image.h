#pragma once

namespace hl_monitoring
{

/**
 * Represents an image along with its intrinsic and extrinsic parameters
 */
class CalibratedImage {
public:

private:
  cv::Mat img;
  cv::Mat camera_matrix;
  cv::Mat camera_distortion_coeffs;
  cv::Mat rvec;
  cv::Mat tvec;
};

}
