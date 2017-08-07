#ifndef PLANNER_H
#define PLANNER_H

#include <list>
#include <vector>

#include "types.h"
#include "map.h"
#include "vehicle.h"
#include "behavior.h"
#include "trajectory.h"

class Planner {

private:
  Map map_;
  std::list<Vehicle> vehicles_;
  Behavior behavior_;
  Trajectory trajectory_;

public:

  Planner(const std::string &map_file);
  ~Planner();

  TrajectoryXY Update(
    Localization &localization,
    std::vector<SensorFusion> &sf_data,
    TrajectoryXY &previous_trajectory);

};

#endif // PLANNER_H
