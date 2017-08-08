#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "types.h"
#include "map.h"

class Trajectory {

private:
  static const size_t kNumberOfPoints = 200;
  const double kTimeBetweenPoints = 0.02;
  const double kMaxLongitudinalAcceleration = 4; // Assuming 0 - 16m/s in 4 seconds
  const double kMinLongitudinalAcceleration = -20; // Assuming 20m/s - 0 in 1 second

  class FrenetPointBuffer {
  private:
    FrenetPoint points_[kNumberOfPoints];
    int front_ = 0;

  public:
    FrenetPointBuffer() {}
    ~FrenetPointBuffer() {}

    void append(FrenetPoint value) {
        points_[front_] = value;
        front_ = (front_ + 1) % kNumberOfPoints;
    }

    FrenetPoint& operator [] (int index) {
        if (index < 0) {
          index += kNumberOfPoints;
        }
        int loc = (front_ + index) % kNumberOfPoints;
        return points_[loc];
    }
  };

  FrenetPointBuffer sd_points_;

  std::vector<double> JMT(std::vector<double> start, std::vector<double> end, double t);
  double PolyEval(std::vector<double> &coeffs, double x);

public:

  Trajectory();
  ~Trajectory();

  void Reset();

  TrajectoryXY Generate(
    Map &map,
    Localization &localization,
    TrajectoryXY &previous_trajectory,
    FrenetPoint &previous_coordinates,
    BehaviorInfo &behavior);

};

#endif // TRAJECTORY_H
