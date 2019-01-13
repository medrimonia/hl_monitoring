#include "hl_monitoring/field.h"

#include "hl_monitoring/utils.h"

namespace hl_monitoring {

static double read(const Json::Value & v, const std::string & key) {  
  if (!v.isObject()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " v is not an object");
  }
  if (!v.isMember(key)) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " v has no key '" + key + "'");
  }
  if (!v.isDouble()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " v[" + key + "] is not a double");
  }
  return v.asDouble();
}

Field::Field() {
  ballRadius = 0.075;
  /// Expected field sizes
  lineWidth         = 0.05;
  centerRadius      = 0.75;
  borderStripWidth  = 0.70;//Minimum value
  penaltyMarkDist   = 2.10;
  penaltyMarkLength = 0.10;
  goalWidth         = 2.60;
  goalDepth         = 0.60;
  goalAreaLength    = 1.00;
  goalAreaWidth     = 5.00;
  fieldLength       = 9.00;
  fieldWidth        = 6.00;
}

Json::Value Field::toJson() const {
  Json::Value v;
  v["ballRadius"       ] = ballRadius        ;
  v["lineWidth"        ] = lineWidth         ;
  v["centerRadius"     ] = centerRadius      ;
  v["borderStripWidth" ] = borderStripWidth  ;
  v["penaltyMarkDist"  ] = penaltyMarkDist   ;
  v["penaltyMarkLength"] = penaltyMarkLength ;
  v["goalWidth"        ] = goalWidth         ;
  v["goalDepth"        ] = goalDepth         ;
  v["goalAreaLength"   ] = goalAreaLength    ;
  v["goalAreaWidth"    ] = goalAreaWidth     ;
  v["fieldLength"      ] = fieldLength       ;
  v["fieldWidth"       ] = fieldWidth        ;
  return v;
}

void Field::fromJson(const Json::Value & v) {
  ballRadius        = read(v,"ballRadius"       );
  lineWidth         = read(v,"lineWidth"        );
  centerRadius      = read(v,"centerRadius"     );
  borderStripWidth  = read(v,"borderStripWidth" );
  penaltyMarkDist   = read(v,"penaltyMarkDist"  );
  penaltyMarkLength = read(v,"penaltyMarkLength");
  goalWidth         = read(v,"goalWidth"        );
  goalDepth         = read(v,"goalDepth"        );
  goalAreaLength    = read(v,"goalAreaLength"   );
  goalAreaWidth     = read(v,"goalAreaWidth"    );
  fieldLength       = read(v,"fieldLength"      );
  fieldWidth        = read(v,"fieldWidth"       );
}

}
