#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <list>

#include "types.h"
#include "map.h"
#include "vehicle.h"

class Behavior {

private:
  State state_;
  Lane target_lane_;

  Lane SelectBestLane(Lane current_lane, std::list<Vehicle> &vehicles);

public:

  Behavior();
  ~Behavior();

  std::pair<State,Lane> Update(Map &map, Localization &localization, std::list<Vehicle> &vehicles);

};

#endif // BEHAVIOR_H
