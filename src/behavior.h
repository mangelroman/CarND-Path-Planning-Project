#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <list>

#include "types.h"
#include "map.h"

class Behavior {

private:
  const double kMaxTargetSpeed = 22;
  const double kMinimumFrontGap = 30;
  const double kMinimumBackGap = 20;
  const double kChangeLaneTolerance = 0.3;
  const double kChangeSpeedTolerance = 0.5;

  const double kLaneScoreFrontGapWeight = 1000;
  const double kLaneScoreBackGapWeight = 100;
  const double kLaneScoreSpeedWeight = 1;
  const double kLaneScoreFarLaneWeight = 1000;

  const double kChangeLaneCostMargin = 5;

  BehaviorInfo info_;

  Lane GetBestLane(Map &map, Localization &loc, std::vector<VehicleInfo> &vehicles);

public:

  Behavior();
  ~Behavior();

  void Reset();

  BehaviorInfo Update(Map &map, Localization &loc, std::vector<VehicleInfo> &vehicles);

};

#endif // BEHAVIOR_H
