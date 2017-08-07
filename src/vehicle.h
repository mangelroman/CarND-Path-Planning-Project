#ifndef VEHICLE_H
#define VEHICLE_H

#include <queue>

#include "types.h"

class Vehicle {

private:
  const double  kChangePredictionThreshold = 1;
  const int     kMaxTrajectoryPoints = 50;

  std::queue<std::pair<double,double>> trajectory_;

  int id_;
  double velocity_;
  bool visible_;

public:

  Vehicle(int id);
  ~Vehicle();

  void Update(SensorFusion &sensor_fusion);
  Prediction Predict();

  int GetId() const { return id_; }

  double GetVelocity() const { return velocity_; }

  void ClearVisible() { visible_ = false; }
  bool IsVisible() const { return visible_; }
};

#endif // VEHICLE_H
