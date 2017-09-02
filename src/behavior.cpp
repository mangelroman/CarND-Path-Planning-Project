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

Lane Behavior::GetBestLane(Map &map, Localization &loc, std::vector<VehicleInfo> &vehicles) {
  Lane current_lane = map.GetLane(loc.d);

  std::map<Lane,double> cost;

  cost[Lane::kOutLeft] = 100000;
  cost[Lane::kLeft] = (current_lane == Lane::kRight) ? kLaneScoreFarLaneWeight : 10;
  cost[Lane::kCenter] = 0;
  cost[Lane::kRight] = (current_lane == Lane::kLeft) ? kLaneScoreFarLaneWeight : 10;
  cost[Lane::kOutRight] = 100000;

  for (auto vehicle : vehicles) {
    double speed_diff = loc.v - vehicle.speed;

    if (vehicle.distance > 0) {
      // Object is close by
      cost[vehicle.lane] += kLaneScoreFrontGapWeight / vehicle.distance;
      cost[vehicle.lane] += kLaneScoreSpeedWeight * speed_diff;
    }
    else {
      if (current_lane != vehicle.lane) {
        // Object is not behind our lane
        cost[vehicle.lane] += kLaneScoreBackGapWeight / (-vehicle.distance);
        cost[vehicle.lane] += kLaneScoreSpeedWeight * (-speed_diff);
        cost[vehicle.lane] += kChangeLaneCostMargin;
      }
    }
  }

  auto best_lane = min_element(cost.begin(), cost.end(), [] (const pair<Lane,double> & p1, const pair<Lane,double> & p2) {
        return p1.second < p2.second;
  })->first;

  // Check feasibility
  double front_gap = 10000, back_gap = 10000;
  for (auto vehicle : vehicles) {
    if (vehicle.lane == best_lane) {
      if (vehicle.distance > 0) {
        // Front vehicle
        double gap = vehicle.distance - ((loc.v + kMaxTargetSpeed) / 2 - vehicle.speed) * 3;
        if (gap < front_gap) {
          front_gap = gap;
        }
      }
      else {
        // Back vehicle
        double gap = -vehicle.distance + ((loc.v + kMaxTargetSpeed) / 2 - vehicle.speed) * 3;
        if (gap < back_gap) {
          back_gap = gap;
        }
      }
    }
  }

  //cout << "BESTLANE=" << int(best_lane) << " FGAP=" << front_gap << " BGAP=" << back_gap << endl;
  if ((front_gap > kMinimumFrontGap) && (back_gap > kMinimumBackGap)) {
    return best_lane;
  }

  return current_lane;
}

BehaviorInfo Behavior::Update(Map &map, Localization &loc, std::vector<VehicleInfo> &vehicles) {

  Lane current_lane = map.GetLane(loc.d);

  switch(info_.state) {

    case State::kStop:
      info_.state = State::kStart;
      info_.target_speed = kMaxTargetSpeed;
      info_.target_lane = current_lane;
      break;

    case State::kStart:
      info_.state = State::kKeepLane;
      break;

    case State::kKeepLane: {
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
          // Not changing Lanes
          Lane best_lane = GetBestLane(map, loc, vehicles);
          if (best_lane != current_lane) {
            info_.target_lane = best_lane;
            info_.state = State::kChangeLane;
            info_.target_speed = kMaxTargetSpeed;
          }
          else {
            info_.target_speed = kMaxTargetSpeed;
            if (abs(loc.v - kMaxTargetSpeed) > kChangeSpeedTolerance) {
              info_.state = State::kChangeSpeed;
            }
            for (auto obj : vehicles) {
              if (obj.lane == current_lane) {
                // Object is in our lane
                if ((obj.distance < kMinimumFrontGap) && (obj.distance > 0) && (obj.speed < kMaxTargetSpeed)) {
                  // Object is blocking us
                  info_.target_speed = obj.speed;
                  info_.state = State::kChangeSpeed;
                  // Exit for loop
                  break;
                }
              }
            }
          }
        }

      }
      break;

    case State::kChangeLane:
    case State::kChangeSpeed:
      info_.state = State::kKeepLane;
      break;
  }

  return info_;
}
