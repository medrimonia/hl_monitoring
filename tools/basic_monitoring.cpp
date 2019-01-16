/**
 * Acquire video and Game messages from a monitoring manager and displays them on screen.
 *
 * Depending on configuration of image providers, video streams and
 * meta_information are written
 */
#include <hl_communication/utils.h>
#include <hl_monitoring/monitoring_manager.h>
#include <hl_monitoring/utils.h>

#include <opencv2/highgui.hpp>
#include <tclap/CmdLine.h>

using namespace hl_communication;
using namespace hl_monitoring;

int main(int argc, char ** argv) {
  TCLAP::CmdLine cmd("Acquire and display one or multiple streams along with meta-information",
                     ' ', "0.9");

  TCLAP::ValueArg<std::string> config_arg("c", "config", "The path to the json configuration file",
                                          true, "config.json", "string");
  cmd.add(config_arg);

  try {
    cmd.parse(argc, argv);
    
  } catch (const TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  MonitoringManager manager;

  manager.loadConfig(config_arg.getValue());

  // While exit was not explicitly required, run
  uint64_t now = 0;
  uint64_t dt = 30 * 1000;//[microseconds]
  if (!manager.isLive()) {
    now = manager.getStart();
  }
  while(manager.isGood()) {
    manager.update();
    if (manager.isLive()) {
      now = getTimeStamp();
    } else {
      now += dt;
    }

    std::cout << "Time: " << std::setprecision(15) << now << std::endl;
    MessageManager::Status status = manager.getStatus(now);
    for (const auto & robot_entry : status.robot_messages) {
      std::cout << "-> Message from robot " << robot_entry.first.robot_id()
                << " from team " << robot_entry.first.team_id() << std::endl;
    }
    
    std::map<std::string, CalibratedImage> images_by_source =
      manager.getCalibratedImages(now);
    for (const auto & entry : images_by_source) {
      cv::Mat display_img = entry.second.getImg().clone();
      cv::imshow(entry.first, display_img);
    }
    char key = cv::waitKey(10);
    if (key == 'q' || key == 'Q') break;
  }
}
