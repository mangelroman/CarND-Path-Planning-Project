#ifndef VEHICLE_H
#define VEHICLE_H

#include <list>

#include "types.h"

class Vehicle {

private:
  std::list<std::pair<double,double>> trajectory_;

public:

  Vehicle();
  ~Vehicle();

  void Update(SensorFusion &sensor_fusion);
  Prediction Predict();
  Lane GetLane();
};

#endif // VEHICLE_H
