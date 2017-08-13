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
  info_.state = State::kStart;
  info_.target_lane = Lane::kCenter;
  info_.target_speed = 0;
}

BehaviorInfo Behavior::Update(Map &map, Localization &loc, std::vector<SensorFusion> &sensor_fusion, Lane best_lane) {

  Lane current_lane = map.GetLane(loc.d);

  switch(info_.state) {
    case State::kStart:
      info_.state = State::kChangeSpeed;
      info_.target_lane = current_lane;
      info_.target_speed = kMaxTargetSpeed;
      break;

    case State::kChangeSpeed:
      info_.state = (info_.target_speed == 0) ? State::kStop : State::kKeepLane;
      break;

    case State::kKeepLane:

      if (current_lane == Lane::kOutLeft) {
        info_.target_lane = Lane::kLeft;
        info_.state = State::kChangeLane;
        break;
      }
      if (current_lane == Lane::kOutRight) {
        info_.target_lane = Lane::kRight;
        info_.state = State::kChangeLane;
        break;
      }

      if (abs(loc.d - map.GetCenterOfLane(info_.target_lane)) < kChangeLaneTolerance) {
        // Not in the middle of changing lanes
        info_.target_speed = kMaxTargetSpeed;
        for (auto obj : sensor_fusion) {
          if (map.GetLane(obj.d) == current_lane) {
            // Object is in our lane
            double distance = map.ComputeDistance(loc.s, obj.s);
            double obj_speed = sqrt(pow(obj.vx, 2) + pow(obj.vy, 2));

            if ((distance < kMinimumVehicleDistance) && (distance > 0) && (obj_speed < kMaxTargetSpeed)) {
              // Object is blocking us
              if (best_lane != current_lane) {
                info_.target_lane = best_lane;
                info_.state = State::kChangeLane;
              }
              else {
                info_.target_speed = obj_speed;
                info_.state = State::kChangeSpeed;
              }
              // Exit for loop
              break;
            }
          }
        }
      }
      break;

    case State::kChangeLane:
      info_.state = State::kKeepLane;
      break;

    case State::kStop:
      info_.state = State::kStop;
      break;

    case State::kEmergencyBreak:
      info_.state = State::kStop;
      info_.target_lane = current_lane;
      info_.target_speed = 0;
      break;
  }

  return info_;
}
