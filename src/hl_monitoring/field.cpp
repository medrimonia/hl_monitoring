#include "hl_monitoring/field.h"

#include "hl_monitoring/utils.h"

#include <fstream>

namespace hl_monitoring {

static double read(const Json::Value & v, const std::string & key) {  
  if (!v.isObject()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " v is not an object");
  }
  if (!v.isMember(key)) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " v has no key '" + key + "'");
  }
  if (!v[key].isDouble()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " v[" + key + "] is not a double");
  }
  return v[key].asDouble();
}

Field::Field() {
  ball_radius = 0.075;
  /// Expected field sizes
  line_width          = 0.05;
  center_radius       = 0.75;
  border_strip_width  = 0.70;//Minimum value
  penalty_mark_dist   = 2.10;
  penalty_mark_length = 0.10;
  goal_width          = 2.60;
  goal_depth          = 0.60;
  goal_area_length    = 1.00;
  goal_area_width     = 5.00;
  field_length        = 9.00;
  field_width         = 6.00;
}

Json::Value Field::toJson() const {
  Json::Value v;
  v["ball_radius"        ] = ball_radius         ;
  v["line_width"         ] = line_width          ;
  v["center_radius"      ] = center_radius       ;
  v["border_strip_width" ] = border_strip_width  ;
  v["penalty_mark_dist"  ] = penalty_mark_dist   ;
  v["penalty_mark_length"] = penalty_mark_length ;
  v["goal_width"         ] = goal_width          ;
  v["goal_depth"         ] = goal_depth          ;
  v["goal_area_length"   ] = goal_area_length    ;
  v["goal_area_width"    ] = goal_area_width     ;
  v["field_length"       ] = field_length        ;
  v["field_width"        ] = field_width         ;
  return v;
}

void Field::fromJson(const Json::Value & v) {
  ball_radius         = read(v,"ball_radius"        );
  line_width          = read(v,"line_width"         );
  center_radius       = read(v,"center_radius"      );
  border_strip_width  = read(v,"border_strip_width" );
  penalty_mark_dist   = read(v,"penalty_mark_dist"  );
  penalty_mark_length = read(v,"penalty_mark_length");
  goal_width          = read(v,"goal_width"         );
  goal_depth          = read(v,"goal_depth"         );
  goal_area_length    = read(v,"goal_area_length"   );
  goal_area_width     = read(v,"goal_area_width"    );
  field_length        = read(v,"field_length"       );
  field_width         = read(v,"field_width"        );
  updatePointsOfInterest();
}

void Field::loadFile(const std::string & path) {
  std::ifstream in(path);
  if (!in.good()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " failed to open file '" + path + "'");
  }
  Json::Value root;
  in >> root;
  fromJson(root);
}

const std::map<std::string, cv::Point3f> & Field::getPointsOfInterest() const {
  return points_of_interest;
}

void Field::updatePointsOfInterest() {
  points_of_interest.clear();
  points_of_interest["center"] = cv::Point3f(0, 0, 0);
  double fc_x = field_length/2;
  double fc_y = field_width/2;
  points_of_interest["field_corner++"] = cv::Point3f( fc_x,  fc_y, 0);
  points_of_interest["field_corner+-"] = cv::Point3f( fc_x, -fc_y, 0);
  points_of_interest["field_corner-+"] = cv::Point3f(-fc_x,  fc_y, 0);
  points_of_interest["field_corner--"] = cv::Point3f(-fc_x, -fc_y, 0);
  double gac_x = field_length / 2 - goal_area_width;
  double gac_y = field_width / 2 - goal_area_length;
  points_of_interest["goal_area_corner++"] = cv::Point3f( gac_x,  gac_y, 0);
  points_of_interest["goal_area_corner+-"] = cv::Point3f( gac_x, -gac_y, 0);
  points_of_interest["goal_area_corner-+"] = cv::Point3f(-gac_x,  gac_y, 0);
  points_of_interest["goal_area_corner--"] = cv::Point3f(-gac_x, -gac_y, 0);
  double pm_x = field_length / 2 - penalty_mark_dist;
  points_of_interest["penalty_mark+"] = cv::Point3f( pm_x, 0, 0);
  points_of_interest["penalty_mark-"] = cv::Point3f(-pm_x, 0, 0);
  double mlt_y = field_width / 2;
  points_of_interest["middle_line_t+"] = cv::Point3f(0,  mlt_y, 0);
  points_of_interest["middle_line_t-"] = cv::Point3f(0, -mlt_y, 0);
}

}
