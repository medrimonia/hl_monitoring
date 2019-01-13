#pragma once

#include <json/json.h>

namespace hl_monitoring {

/**
 * Contains all the information about the size of the field, also allows to
 * retrieve points of interest
 *
 *
 * Referential is as follows:
 * - Zero: Center of field at ground level
 * - X-axis goes from center of the field to the center of the post at the right side of the team area
 * - Y-axis goes from center toward side line opposite to team area
 * - Z-axis points toward the roof
 *
 *                         y
 *  |----------------------------------------------|
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      0--->x                  |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |----------------------------------------------|
 *
 *         |--------------------------------|
 *         |                                |
 *         |           Team area            |
 *         |                                |
 *         |--------------------------------|
 *
 * - All field informations are in meters
 */
class Field {
public:
  Field();

  Json::Value toJson() const;
  void fromJson(const Json::Value & v);

  /// Radius of the ball [m]
  double ballRadius;

  /// The width of white lines [m]
  double lineWidth;

  /// Radius of the central circle (lines included) [m]
  double centerRadius;

  /// Distance from fieldBorder to arenaBorder (line excluded) [m]
  double borderStripWidth;

  /// Distance from center of penaltyMark to closest goal (goal line included) [m]
  double penaltyMarkDist;

  /// Length of the penalty mark [m]
  double penaltyMarkLength;

  /// Distance between two posts (posts excluded) [m]
  double goalWidth;

  /// From goal line to the back of the goal (goal line included) [m]
  double goalDepth;

  /// From goal line to goal area line (lines included) [m]
  double goalAreaLength;

  /// From one side of goal area to the other (lines included) [m]
  double goalAreaWidth;

  /// From one goal line to the other (lines included) [m]
  double fieldLength;

  /// From one side of the field to the other (lines included) [m]
  double fieldWidth;
};

}
