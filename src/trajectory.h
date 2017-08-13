#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "types.h"
#include "map.h"

class Trajectory {

private:
  static const size_t kNumberOfPoints = 200;
  const double kTimeBetweenPoints = 0.02;
  const double kMaxLAcceleration = 5;
  const double kMaxLJerk = 9;
  const double kMaxLDeceleration = -20; // Assuming 20m/s - 0 in 1 second

  template<class T, size_t N> class CircularBuffer {
  private:
    T items_[N];
    int front_;

  public:
    CircularBuffer(): front_(0) { }
    ~CircularBuffer() { }

    void append(T value) {
        items_[front_] = value;
        front_ = (front_ + 1) % N;
    }

    T& operator [] (int index) {
        if (index < 0) {
          index += N;
        }
        int loc = (front_ + index) % N;
        return items_[loc];
    }
  };

  CircularBuffer<FrenetPoint,kNumberOfPoints> sd_points_;
  CircularBuffer<XYPoint,kNumberOfPoints> xy_points_;

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
