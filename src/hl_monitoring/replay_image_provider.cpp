#include "hl_monitoring/replay_image_provider.h"

#include <fstream>

namespace hl_monitoring
{

ReplayImageProvider::ReplayImageProvider() {
}

ReplayImageProvider::ReplayImageProvider(const std::string & video_path,
                                         const std::string & meta_information_path) {
  load(video_path, meta_information_path);
}

void ReplayImageProvider::load(const std::string & video_path,
                               const std::string & meta_information_path) {
  
  if (!video.open(video_path)) {
    throw std::runtime_error("Failed to open video '" + video_path + "'");
  }
  std::ifstream in(meta_information_path, std::ios::binary);
  if (!in.good()) {
    throw std::runtime_error("Failed to open file '" + meta_information_path + "'");
  }
  if (!meta_information.ParseFromIstream(&in)) {
    throw std::runtime_error("Failed to read file '" + meta_information_path + "'");
  }
}

CalibratedImage ReplayImageProvider::getCalibratedImage(double time_stamp) {
  int index = getIndex(time_stamp);

  if (!video.set(cv::CAP_PROP_POS_FRAMES, index)) {
    throw std::runtime_error("Failed to set index to " + std::to_string(index) + " in video");
  }
  cv::Mat img;
  video.read(img);
  if (img.empty()) {
    throw std::runtime_error("Blank frame has been read");
  }
  
  IntrinsicParameters intrinsic_parameters;
  Pose3D camera_pose;
  if (meta_information.has_camera_parameters()) {
    intrinsic_parameters = meta_information.camera_parameters();
  } else {
    throw std::runtime_error("Intrinsic parameters are not available");
  }
  
  const FrameEntry & frame = meta_information.frames(index);
  if (frame.has_pose()) {
    camera_pose = frame.pose();
  } else if (meta_information.has_default_pose()) {
    camera_pose = meta_information.default_pose();
  } else {
    throw std::runtime_error("Frame has no pose information and no default pose available");
  }

  return CalibratedImage(img, camera_pose, intrinsic_parameters);
}

int ReplayImageProvider::getIndex(double time_stamp) const {
  if (indices_by_time_stamp.size() == 0) {
    throw std::runtime_error("indices_by_time_stamp is empty");
  }
  auto it = indices_by_time_stamp.upper_bound(time_stamp);
  if (it == indices_by_time_stamp.end() || it->first > time_stamp) {
    it--;
  }
  return it->second;
}

}
