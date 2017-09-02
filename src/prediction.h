#ifndef VEHICLE_H
#define VEHICLE_H

#include "types.h"
#include "map.h"

class Prediction {

private:

    const double kVisibleFrontDistance = 100;
    const double kVisibleBackDistance = 25;

public:

  Prediction();
  ~Prediction();

  void Reset();

  std::vector<VehicleInfo> Update(Map &map, Localization &loc, SensorFusion &sf_data);

};

#endif // PREDICTION_H
