#pragma once

#include "hl_monitoring/camera.pb.h"

#include <opencv2/core.hpp>

/**
 * Contains multiple conversion tools from hl_communication protobuf format to
 * OpenCV classical format and debug tools
 */

namespace hl_monitoring
{

#define HL_MONITOR_DEBUG                        \
  (std::string(__FUNCTION__) + ":"              \
   + hl_monitoring::getBaseName(__FILE__) + ":" \
   + std::to_string(__LINE__)  + ": ")

/// Return the name of the file at the given path:
/// e.g getBaseName("toto/file.cpp") returns "file.cpp"
std::string getBaseName(const std::string & path);

void intrinsicToCV(const IntrinsicParameters & camera_parameters,
                   cv::Mat * camera_matrix,
                   cv::Mat * distortion_coefficients,
                   cv::Size * img_size);
void cvToIntrinsic(const cv::Mat & camera_matrix,
                   const cv::Mat & distortion_coefficients,
                   const cv::Size & img_size,
                   IntrinsicParameters * camera_parameters);
void pose3DToCV(const Pose3D & pose,
                cv::Mat * rvec,
                cv::Mat * tvec);
void cvToPose3D(const cv::Mat & rvec,
                const cv::Mat & tvec,
                Pose3D * pose);


}
