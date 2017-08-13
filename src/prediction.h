#ifndef VEHICLE_H
#define VEHICLE_H

#include "types.h"
#include "map.h"

class Prediction {

private:
  const double kLaneScoreFrontGapWeight = 1000;
  const double kLaneScoreBackGapWeight = 200;
  const double kLaneScoreSpeedWeight = 1;
  const double kLaneScoreFarLaneWeight = 1000;
  const double kVisibleFrontDistance = 80;
  const double kVisibleBackDistance = 25;
  const double kChangeLaneCostMargin = 5;
public:

  Prediction();
  ~Prediction();

  void Reset();

  Lane GetBestLane(Map &map, Localization &loc, std::vector<SensorFusion> &sensor_fusion);

};

#endif // PREDICTION_H
