#ifndef VEHICLE_H
#define VEHICLE_H

#include <queue>

#include "types.h"
#include "map.h"

class Vehicle {

private:
  const double  kChangePredictionThreshold = 1;
  const int     kMaxTrajectoryPoints = 50;

  std::queue<FrenetPoint> trajectory_;

  int id_;
  double velocity_;
  bool visible_;
  Lane lane_;

public:

  Vehicle(int id);
  ~Vehicle();

  void Update(Map &map, SensorFusion &sensor_fusion);
  Prediction Predict();

  int GetId() const { return id_; }
  double GetVelocity() const { return velocity_; }
  Lane GetLane() const { return lane_; }
  FrenetPoint GetLocation() { return trajectory_.back(); }

  void ClearVisible() { visible_ = false; }
  bool IsVisible() const { return visible_; }
};

#endif // VEHICLE_H
