#pragma once

#include "hl_monitoring/camera.pb.h"

#include <opencv2/core.hpp>

/**
 * Contains multiple conversion tools from hl_communication protobuf format to
 * OpenCV classical format
 */

namespace hl_monitoring
{

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
