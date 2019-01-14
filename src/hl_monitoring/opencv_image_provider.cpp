#include "hl_monitoring/opencv_image_provider.h"

#include "hl_monitoring/utils.h"

#include <chrono>
#include <fstream>

using namespace std::chrono;

namespace hl_monitoring
{

OpenCVImageProvider::OpenCVImageProvider(const std::string & video_path,
                                         const std::string & output_path)
{
  openInputStream(video_path);
  if (output_path != "") {
    openOutputStream(output_path);
  }
}

void OpenCVImageProvider::openInputStream(const std::string & video_path) {
  if (!input.open(video_path)) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " failed to open device '" + video_path + "'");
  }
  int read_width = input.get(cv::CAP_PROP_FRAME_WIDTH);
  int read_height = input.get(cv::CAP_PROP_FRAME_HEIGHT);
  img_size = cv::Size(read_width, read_height);
  if (meta_information.has_camera_parameters()) {
    // Intrinsic parameters have been provided, check consistency
    int expected_width = meta_information.camera_parameters().img_width();
    int expected_height = meta_information.camera_parameters().img_height();
    if (expected_width != read_width || expected_height != read_height) {
      std::ostringstream oss;
      oss << HL_MONITOR_DEBUG << " mismatch of sizes: expecting "
          << expected_width << "*" << expected_height
          << ", stream size " << read_width << "*" << read_height;
      throw std::runtime_error(oss.str());
    }
  }
}

void OpenCVImageProvider::openOutputStream(const std::string & output_path) {
  if (!input.isOpened()) {
    throw std::logic_error(HL_MONITOR_DEBUG + " input stream is not open yet");
  }
  double fps = input.get(cv::CAP_PROP_FPS);
  bool use_color = true;
  output.open(output_path, cv::VideoWriter::fourcc('X','V','I','D'), fps, img_size, use_color);
  if (!output.isOpened()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + "Failed to open video");
  }
}

void OpenCVImageProvider::restartStream() {
  throw std::logic_error("It makes no sense to restart the stream in a 'OpenCVImageProvider'");
}

CalibratedImage OpenCVImageProvider::getCalibratedImage(double time_stamp) {
  if (nb_frames == 0) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " no frames found in the stream");
  }
  if (time_stamp < indices_by_time_stamp.rbegin()->first) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " asking for frames in the past is not supported");
  }

  int index = indices_by_time_stamp.size() - 1;
  
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

cv::Mat OpenCVImageProvider::getNextImg() {
  input.read(img);

  double time_stamp =
    duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
  if (img.empty()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + "Blank frame at frame: "
                             + std::to_string(index) + "/" + std::to_string(nb_frames));
  }
  // register image
  indices_by_time_stamp[time_stamp] = index;
  FrameEntry * entry  = meta_information.add_frames();
  entry->set_time_stamp(time_stamp);
  index++;
  nb_frames++;
  // Write image to output video if opened
  if (output.isOpened()) {
    output.write(img);
  }
  return img;
}

bool OpenCVImageProvider::isStreamFinished() {
  return false;
}


}
