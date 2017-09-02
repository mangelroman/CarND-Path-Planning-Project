#include "prediction.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

Prediction::Prediction() {
}

Prediction::~Prediction() {
}

void Prediction::Reset() {
}

std::vector<VehicleInfo> Prediction::Update(Map &map, Localization &loc, SensorFusion &sf_data) {

  vector<VehicleInfo> vehicles;
  vehicles.reserve(sf_data.size());

  for(auto&& item : sf_data) {

    double distance = map.ComputeDistance(loc.s, item[5]);

    if ((distance < kVisibleFrontDistance) && (distance > -kVisibleBackDistance)) {

      VehicleInfo vehicle;
      vehicle.id = item[0];
      vehicle.x = item[1];
      vehicle.y = item[2];
      vehicle.vx = item[3];
      vehicle.vy = item[4];
      vehicle.s = item[5];
      vehicle.d = item[6];
      vehicle.distance = distance;
      vehicle.speed = sqrt(pow(vehicle.vx, 2) + pow(vehicle.vy, 2));
      vehicle.lane = map.GetLane(vehicle.d);

      vehicles.push_back(vehicle);
    }

  }

  return vehicles;
}
