#pragma once

#include "hl_monitoring/image_provider.h"

#include <opencv2/videoio.hpp>


namespace hl_monitoring
{

/**
 * Use OpenCV standard API to open a video stream
 * - Images read can be directly encoded in a video
 * - Timestamps are based on acquisition time
 */
class OpenCVImageProvider : public ImageProvider {
public:
  /**
   * If output path is not null, write a video to output_path
   */
  OpenCVImageProvider(const std::string & video_path,
                      const std::string & output_path = "");

  void openInputStream(const std::string & video_path);
  void openOutputStream(const std::string & output_path);

//  void setIntrinsic(const IntrinsicParameters & intrinsic);
//  void setDefaultPose(const Pose3D & pose);
  
  void restartStream() override;

  CalibratedImage getCalibratedImage(double time_stamp) override;
  
  cv::Mat getNextImg() override;

  bool isStreamFinished() override;

private:
  /**
   * The video read from the file
   */
  cv::VideoCapture input;
  
  /**
   * The video read from the file
   */
  cv::VideoWriter output;

  /**
   * Information relevant
   */
  VideoMetaInformation meta_information;

  /**
   * Last img read
   */
  cv::Mat img;

  /**
   * Size of the images provided by the camera
   */
  cv::Size img_size;

  /**
   * Provide access to index using time_stamps
   */
  std::map<double, int> indices_by_time_stamp;

  /**
   * Index of the next image read in the video
   */
  int index;

  /**
   * The number of frames in the video
   */
  int nb_frames;
};

}
