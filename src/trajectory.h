#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "types.h"
#include "map.h"

class Trajectory {

private:
  static const size_t kNumberOfPoints = 200;
  const size_t kPointsToChangeLane = 150;
  const double kTimeBetweenPoints = 0.02;
  const double kAvgAcceleration = 5;

  CircularBuffer<FrenetPoint,kNumberOfPoints> sd_points_;

  std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end, double t);
  double PolyEval(std::vector<double> &coeffs, double x);

public:

  Trajectory();
  ~Trajectory();

  void Reset();

  TrajectoryXY Generate(
    Map &map,
    Localization &loc,
    TrajectoryXY &previous_trajectory,
    FrenetPoint &previous_coordinates,
    BehaviorInfo &behavior);

};

#endif // TRAJECTORY_H
