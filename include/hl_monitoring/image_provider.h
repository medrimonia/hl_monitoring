#pragma once

#include "hl_monitoring/calibrated_image.h"

namespace hl_monitoring
{

class ImageProvider {
public:
  
  /**
   * Return an image along with associated intrinsic and extrinsic parameters
   */
  virtual CalibratedImage getCalibratedImage(double time_stamp) = 0;
};

}
