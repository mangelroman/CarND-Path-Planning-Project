#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "types.h"
#include "map.h"

class Trajectory {

private:
  const size_t kNumberOfPoints = 50;

public:

  Trajectory();
  ~Trajectory();

  TrajectoryXY Generate(
    Map &map,
    Localization &localization,
    TrajectoryXY &previous_trajectory,
    std::pair<State,Lane> &behavior);

};

#endif // TRAJECTORY_H
