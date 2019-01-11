#pragma once

namespace hl_monitoring
{

class ImageProvider {
public:

  /**
   * Return the extrinsic and intrinsic
   */
  std::pair<CameraState, cv::Mat> getEntry(double time_stamp) const;
  
private:
  std::map<double, cv::Mat> images;

  CameraState camera_state;
};

}
