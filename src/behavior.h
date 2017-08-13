#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <list>

#include "types.h"
#include "map.h"

class Behavior {

private:
  const double kMaxTargetSpeed = 22;
  const double kMinimumVehicleDistance = 35;
  const double kChangeLaneTolerance = 0.5;

  BehaviorInfo info_;

public:

  Behavior();
  ~Behavior();

  void Reset();

  BehaviorInfo Update(
    Map &map,
    Localization &loc,
    std::vector<SensorFusion> &sensor_fusion,
    Lane best_lane
  );

};

#endif // BEHAVIOR_H
