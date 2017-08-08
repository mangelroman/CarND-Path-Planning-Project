#include "vehicle.h"

#include <cmath>

using namespace std;

Vehicle::Vehicle(int id): visible_(false), velocity_(0), id_(id) {
}

Vehicle::~Vehicle() {
}

void Vehicle::Update(Map &map, SensorFusion &sensor_fusion) {
  // Assume the vehicle does not change lanes, thus v = s_dot
  velocity_ = sqrt(sensor_fusion.vx * sensor_fusion.vx + sensor_fusion.vy * sensor_fusion.vy);
  trajectory_.push(make_pair(sensor_fusion.s, sensor_fusion.d));
  lane_ = map.GetLane(sensor_fusion.d);
  // Keep latest (s,d) trajectory points
  if (trajectory_.size() > kMaxTrajectoryPoints) {
    trajectory_.pop();
  }
  visible_ = true;
}

Prediction Vehicle::Predict() {
  double d0 = trajectory_.front().second;
  double d1 = trajectory_.back().second;
  double diff = d1 - d0;

  // Simple predcition algorithm based on change of d over a number of samples
  if (abs(diff) < kChangePredictionThreshold) {
    return (diff < 0) ? Prediction::kChangeLeft : Prediction::kChangeRight;
  }
  return Prediction::kStraight;
}
