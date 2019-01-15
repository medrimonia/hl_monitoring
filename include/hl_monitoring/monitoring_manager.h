#pragma once

#include <hl_monitoring/image_provider.h>
#include <hl_communication/message_manager.h>

#include <json/json.h>
#include <memory>

namespace hl_monitoring
{

/**
 * Manage the monitoring of a game or a replay for the RoboCup humanoid league
 *
 * Provide access to the following elements:
 * - Status message sent by the robot
 * - Game Controller status
 * - Named video streams with parameters of the cameras (intrinsic+extrinsic)
 */
class MonitoringManager {
public:
  MonitoringManager();

  void loadConfig(const std::string & path);

  std::unique_ptr<ImageProvider> buildImageProvider(const Json::Value & v);

  void loadImageProviders(const Json::Value & v);
  void loadMessageManager(const Json::Value & v);

  void setMessageManager(std::unique_ptr<hl_communication::MessageManager> message_manager);
  void addImageProvider(const std::string & name,
                        std::unique_ptr<ImageProvider> image_provider);

  void update();

  std::map<std::string, CalibratedImage> getCalibratedImages(double time_stamp);
  
  bool isGood() const;

private:
  /**
   * Access to message from both, robots and GameController
   */
  std::unique_ptr<hl_communication::MessageManager> message_manager;

  /**
   * Access to all the channels allowing to retrieve images
   */
  std::map<std::string,std::unique_ptr<ImageProvider>> image_providers;

};

}
