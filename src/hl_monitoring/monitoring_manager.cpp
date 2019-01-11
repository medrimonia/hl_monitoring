#include "hl_monitoring/monitoring_manager.h"

using namespace hl_communication;

namespace hl_monitoring
{

MonitoringManager::MonitoringManager() {
}

void MonitioringManager::setMessageManager(std::unique_ptr<MessageManager> new_message_manager) {
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

}
