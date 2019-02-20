#pragma once

#include "hl_monitoring/image_provider.h"

#include <json/json.h>
#include <flycapture/FlyCapture2.h>
#include <opencv2/videoio.hpp>

namespace hl_monitoring
{

class PtGreyException : public std::runtime_error {
public:
  PtGreyException(const std::string & msg);
};

class PtGreyConnectionException : public std::runtime_error {
public:
  PtGreyConnectionException(const std::string & msg);
};


/**
 * Use 'flycapture' API from FLIR (previously PtGrey) to capture images
 */
class FlyCapImageProvider : public ImageProvider {
public:
  /**
   * If output_prefix is not empty, write video during execution and saves
   * MetaInformation when object is closed
   */
  FlyCapImageProvider(const Json::Value & v,
                      const std::string & output_prefix = "");
  virtual ~FlyCapImageProvider();

  
  /**
   * Cut current connection if active, then start a new connection
   */
  void reconnectCamera();

  void openInputStream();
  void openOutputStream(const std::string & output_path);
  
  void restartStream() override;

  CalibratedImage getCalibratedImage(uint64_t time_stamp) override;

  void update() override;
  
  cv::Mat getNextImg() override;

  bool isStreamFinished() override;

  void saveVideoMetaInformation();

  uint64_t getStart() const override;

  void updateProperty(const FlyCapture2::Property & wished_property);
  void applyWishedProperties();
  
  /**
   * Retrieve current image format
   */
  FlyCapture2::GigEImageSettings getImageSettings();

  /*
   * Update the packet size and the packet delay to reduce image
   * inconsistencies
   * NOTE: this has to be done while the camera is not capturing
   */
  void updatePacketProperties();

  /**
   * Update the imaging mode, binning and PixelFormat of the camera
   */
  void updateImageSettings();

  
  void setImagingMode(FlyCapture2::Mode mode);

  /*
   * Update binning properties if required
   * WARNING: updating binning can change the current imaging mode
   */
  void updateBinning(unsigned int h_binning, unsigned int v_binning);

  /**
   * Set current pixel format without changing other properties
   * Note: dump(getImageSettingsInfo(), std::cout) to see available formats
   */
  void setPixelFormat(FlyCapture2::PixelFormat pixel_format);

  /**
   * Add/replace parameters of the camera
   */
  void setIntrinsic(const IntrinsicParameters & params) override;
  void setDefaultPose(const Pose3D & pose) override;
private:
  /**
   * PtGrey camera
   */
  FlyCapture2::GigECamera camera;

  /**
   * Is the image stream started
   */
  bool is_capturing;
  
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
  std::map<uint64_t, int> indices_by_time_stamp;

  /**
   * Index of the next image read in the video
   */
  int index;

  /**
   * The number of frames in the video
   */
  int nb_frames;

  /**
   * Required frame_rate
   */
  double frame_rate;

  /**
   * Shutter duration [ms]
   */
  double shutter;
  
  /**
   * Camera gain
   */
  double gain;

  /**
   * The prefix used for writing video file and meta_information file. If empty,
   * then no files are written
   */
  std::string output_prefix;
};

}
