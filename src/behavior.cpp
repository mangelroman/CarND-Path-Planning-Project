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
  double current_gap = 10000;
  for (auto vehicle : vehicles) {
    if (vehicle.lane == best_lane) {
      if (vehicle.distance > 0) {
        // Front vehicle in the target lane
        double gap = vehicle.distance - ((loc.v + kMaxTargetSpeed) / 2 - vehicle.speed) * kChangeLaneGapPeriod;
        if (gap < front_gap) {
          front_gap = gap;
        }
      }
      else {
        // Back vehicle in the target lane
        double gap = -vehicle.distance + ((loc.v + kMaxTargetSpeed) / 2 - vehicle.speed) * kChangeLaneGapPeriod;
        if (gap < back_gap) {
          back_gap = gap;
        }
      }
    }
    else if ((vehicle.lane == current_lane) && (vehicle.distance > 0)) {
      // Avoid collision with the front vehicle in the current lane
      double gap = vehicle.distance - ((loc.v + kMaxTargetSpeed) / 2 - vehicle.speed) * kChangeLaneGapPeriod;
      if (gap < current_gap) {
        current_gap = gap;
      }
    }
  }

  cout << "BESTLANE=" << int(best_lane) << " FGAP=" << front_gap << " BGAP=" << back_gap << " CGAP=" << current_gap << endl;
  if ((front_gap > kChangeLaneMinimumFrontGap) &&
      (back_gap > kChangeLaneMinimumBackGap) &&
      (current_gap >= 0)) {
    return best_lane;
  }

  return current_lane;
}

BehaviorInfo Behavior::Update(Map &map, Localization &loc, std::vector<VehicleInfo> &vehicles) {

  Lane current_lane = map.GetLane(loc.d);

  switch(info_.state) {

    case State::kStop:
      info_.state = State::kStart;
      info_.target_speed = kStartTargetSpeed;
      info_.target_lane = current_lane;
      break;

    case State::kStart:
      info_.state = State::kKeepLane;
      break;

    case State::kChangeSpeed:
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
          else {
            info_.state = State::kKeepLane;
          }
          for (auto obj : vehicles) {
            if ((obj.distance > 0) && (obj.lane == current_lane)) {
              // Object is in our lane
              double front_gap = obj.distance - (loc.v - obj.speed); // Gap after 1 second
              if (front_gap < kMinimumFrontGap + 10) {
                info_.target_speed = obj.speed - kReduceSpeedFactor * (kMinimumFrontGap - obj.distance);
                info_.state = State::kChangeSpeed;
              }
              break;
            }
          }
        }
      }
      break;

    case State::kChangeLane:
      info_.state = State::kChangingLanes;
      break;

    case State::kChangingLanes:
      if (abs(loc.d - map.GetCenterOfLane(info_.target_lane)) < kChangeLaneTolerance) {
        info_.state = State::kKeepLane;
      }
      break;
  }

  return info_;
}
