#pragma once

#include "hl_monitoring/image_provider.h"

#include <opencv2/videoio.hpp>


namespace hl_monitoring
{

class ReplayImageProvider : public ImageProvider {
public:

  ReplayImageProvider();
  ReplayImageProvider(const std::string & video_path,
                      const std::string & meta_information_path);
  
  void load(const std::string & video_path,
            const std::string & meta_information_path);

  CalibratedImage getCalibratedImage(double time_stamp) override;

  int getIndex(double time_stamp) const;

private:
  /**
   * The video read from the file
   */
  cv::VideoCapture video;

  /**
   * Information relevant
   */
  VideoMetaInformation meta_information;

  /**
   * Provide access to index using time_stamps
   */
  std::map<double, int> indices_by_time_stamp;
};

}
