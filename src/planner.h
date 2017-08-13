#ifndef PLANNER_H
#define PLANNER_H

#include <list>
#include <vector>

#include "types.h"
#include "map.h"
#include "prediction.h"
#include "behavior.h"
#include "trajectory.h"

class Planner {

private:
  Map map_;
  Prediction prediction_;
  Behavior behavior_;
  Trajectory trajectory_;
  int tick_;

public:

  Planner(const std::string &map_file);
  ~Planner();

  void Reset();

  TrajectoryXY Update(
    Localization &loc,
    std::vector<SensorFusion> &sf_data,
    TrajectoryXY &previous_trajectory,
    FrenetPoint &previous_coordinates
  );

};

#endif // PLANNER_H
