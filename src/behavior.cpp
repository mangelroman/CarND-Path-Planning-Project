#include "behavior.h"

#include <iostream>
#include <cmath>
#include <map>

using namespace std;

Behavior::Behavior() {
  Reset();
}

Behavior::~Behavior() {
}

void Behavior::Reset() {
  info_.state = State::kStop;
  info_.target_lane = Lane::kCenter;
  info_.target_speed = 0;
}

void Behavior::SelectTargets(Map &map, Localization &localization, std::list<Vehicle> &vehicles) {

  Lane current_lane = map.GetLane(localization.d);

  switch(current_lane) {
    case Lane::kOutLeft:
      info_.target_lane = Lane::kLeft;
      return;

    case Lane::kOutRight:
      info_.target_lane = Lane::kRight;
      return;

    default:
      break;
  }

  for (auto vehicle : vehicles) {
    if (vehicle.GetLane() == current_lane) {
      auto vehicle_sd = vehicle.GetLocation();
      double distance = vehicle_sd.first - localization.s;

      if ((distance < kMinimumVehicleDistance) && (distance > 0) && (vehicle.GetVelocity() < localization.v)) {
        switch(current_lane) {
          case Lane::kOutLeft:
            info_.target_lane = Lane::kLeft;
            break;

          case Lane::kLeft:
            info_.target_lane = Lane::kCenter;
            break;

          case Lane::kCenter:
            info_.target_lane = Lane::kLeft;
            break;

          case Lane::kRight:
            info_.target_lane = Lane::kCenter;
            break;

          case Lane::kOutRight:
            info_.target_lane = Lane::kRight;
            break;
        }
      }
    }
  }
}

BehaviorInfo Behavior::Update(Map &map, Localization &localization, std::list<Vehicle> &vehicles) {

  Lane current_lane = map.GetLane(localization.d);

  switch(info_.state) {
    case State::kStop:
      info_.state = State::kStart;
      info_.target_lane = current_lane;
      info_.target_speed = kMaxTargetSpeed;
      break;

    case State::kStart:
      info_.state = State::kKeepLane;
      info_.target_lane = current_lane;
      info_.target_speed = kMaxTargetSpeed;
      break;

    case State::kKeepLane:
      SelectTargets(map, localization, vehicles);
      if (info_.target_lane != current_lane) {
        info_.state = State::kPrepareChangeLane;
      }
      break;

    case State::kPrepareChangeLane:
      info_.state = State::kChangeLane;
      break;

    case State::kChangeLane:
      if (current_lane == info_.target_lane) {
        info_.state = State::kKeepLane;
      }
      break;
  }

  return info_;
}
