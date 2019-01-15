#pragma once

#include "hl_monitoring/camera.pb.h"

#include <json/json.h>
#include <opencv2/core.hpp>

/**
 * Contains multiple conversion tools from hl_communication protobuf format to
 * OpenCV classical format and other utilities functions
 */

namespace hl_monitoring
{

#define HL_MONITOR_DEBUG                        \
  (std::string(__FUNCTION__) + ":"              \
   + hl_monitoring::getBaseName(__FILE__) + ":" \
   + std::to_string(__LINE__)  + ": ")

/**
 * Return the name of the file at the given path:
 * e.g getBaseName("toto/file.cpp") returns "file.cpp"
 */
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

/**
 * Return time_since_epoch in a double value (unit: seconds)
 */
double getTimeStamp();

void checkMember(const Json::Value & v, const std::string & key);

/**
 * Place v[key] in 'dst'
 * Throws an error with an explicit message in case:
 * - v is not an object
 * - v[key] does not contain an object
 * - v[key] has not the required type
 */
template <typename T>
void readVal(const Json::Value & v, const std::string & key, T * dst) = delete;

template <>
void readVal<int>(const Json::Value & v, const std::string & key, int * dst);

template <>
void readVal<double>(const Json::Value & v, const std::string & key, double * dst);

template <>
void readVal<std::string>(const Json::Value & v, const std::string & key, std::string * dst);

/**
 * Place v[key] in 'dst', if v is not an object or if v does not contain key, do
 * not change dst and returns
 *
 * Throws an error with an explicit message in case:
 * - v[key] has not the required type
 */
template <typename T>
void tryReadVal(const Json::Value & v, const std::string & key, T * dst) {
  if (!v.isObject() || !v.isMember(key)) { return; }
  readVal(v, key, dst);
}



}
