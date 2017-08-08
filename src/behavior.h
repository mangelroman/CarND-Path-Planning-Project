#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <list>

#include "types.h"
#include "map.h"
#include "vehicle.h"

class Behavior {

private:
  const double kMaxTargetSpeed = 22;
  const double kMinimumVehicleDistance = 30;

  BehaviorInfo info_;

  void SelectTargets(Map &map, Localization &localization, std::list<Vehicle> &vehicles);

public:

  Behavior();
  ~Behavior();

  void Reset();

  BehaviorInfo Update(Map &map, Localization &localization, std::list<Vehicle> &vehicles);

};

#endif // BEHAVIOR_H
