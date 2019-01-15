#include "hl_monitoring/monitoring_manager.h"

#include <hl_monitoring/opencv_image_provider.h>
#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/utils.h>

#include <fstream>

using namespace hl_communication;

namespace hl_monitoring
{

MonitoringManager::MonitoringManager() {
}

void MonitoringManager::loadConfig(const std::string & path) {
  // Reading Json file
  std::ifstream in(path);
  if (!in.good()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " failed to open file '" + path + "'");
  }
  Json::Value root;
  in >> root;
  // Parsing json content
  checkMember(root, "image_providers");
  checkMember(root, "message_manager");
  loadImageProviders(root["image_providers"]);
  loadMessageManager(root["message_manager"]);
}

std::unique_ptr<ImageProvider> MonitoringManager::buildImageProvider(const Json::Value & v) {
  checkMember(v, "class_name");
  checkMember(v, "input_path");
  std::unique_ptr<ImageProvider> result;
  std::string class_name, input_path;
  readVal(v,"class_name", &class_name);
  readVal(v,"input_path", &input_path);
  if (class_name == "OpenCVImageProvider") {
    std::string output_path;
    if (v.isMember("output_path")) {
      result.reset(new OpenCVImageProvider(input_path, v["output_path"].asString()));
    } else { 
      result.reset(new OpenCVImageProvider(input_path));
    }
  } else if (class_name == "ReplayImageProvider") {
    std::string meta_information_path;
    if (v.isMember("meta_information_path")) {
      result.reset(new ReplayImageProvider(input_path, v["meta_information_path"].asString()));
    } else {
      result.reset(new ReplayImageProvider(input_path));
    }
  }
  return result;
}

void MonitoringManager::loadImageProviders(const Json::Value & v) {
  if (!v.isObject()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " invalid type for v, expecting an object");
  }
  for (Json::ValueConstIterator it = v.begin(); it != v.end(); it++) {
    const std::string & key = it.name();
    addImageProvider(key, buildImageProvider(v[key]));
  }
}

void MonitoringManager::loadMessageManager(const Json::Value & v) {
  if (!v.isObject()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " invalid type for v, expecting an object");
  }
  std::string file_path;
  int port_read = -1;
  tryReadVal(v, "file_path", &file_path);
  tryReadVal(v, "port_read", &port_read);
  bool port_read_set = port_read != -1;
  bool file_path_set = file_path != "";
  if (!port_read_set && !file_path_set) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " neither 'port_read' nor 'file_path' provided");
  } else if (port_read_set && file_path_set) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " both 'port_read' and 'file_path' provided");
  } if (port_read_set) {
    message_manager.reset(new MessageManager(port_read));
  } else {
    message_manager.reset(new MessageManager(file_path));
  }
}

void MonitoringManager::setMessageManager(std::unique_ptr<MessageManager> new_message_manager) {
  message_manager = std::move(new_message_manager);
}

void MonitoringManager::addImageProvider(const std::string & name,
                                         std::unique_ptr<ImageProvider> image_provider) {
  if (image_providers.count(name) > 0) {
    throw std::logic_error("Failed to add Image Provider: '" + name
                           + "' already in collection");
  }
  image_providers[name] = std::move(image_provider);
}

void MonitoringManager::update() {
  for (const auto & entry : image_providers) {
    entry.second->update();
  }
  message_manager->update();
}

std::map<std::string, CalibratedImage>
MonitoringManager::getCalibratedImages(double time_stamp) {
  std::map<std::string, CalibratedImage> images;
  for (const auto & entry : image_providers) {
    images[entry.first] = entry.second->getCalibratedImage(time_stamp);
  }
  return images;
}

bool MonitoringManager::isGood() const {
  for (const auto & entry : image_providers) {
    if (entry.second->isStreamFinished())
      return false;
  }
  return true;
}

}
