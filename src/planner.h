#ifndef PLANNER_H
#define PLANNER_H

#include <list>
#include <vector>

#include "types.h"
#include "map.h"
#include "vehicle.h"
#include "behavior.h"
#include "trajectory.h"

// TODO: take into account s discontinuity

class Planner {

private:
  Map map_;
  std::list<Vehicle> vehicles_;
  Behavior behavior_;
  Trajectory trajectory_;
  int tick_;

public:

  Planner(const std::string &map_file);
  ~Planner();

  void Reset();

  TrajectoryXY Update(
    Localization &localization,
    std::vector<SensorFusion> &sf_data,
    TrajectoryXY &previous_trajectory,
    FrenetPoint &previous_coordinates
  );

};

#endif // PLANNER_H
