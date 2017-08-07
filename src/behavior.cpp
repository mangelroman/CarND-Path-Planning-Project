#include "behavior.h"

#include <cmath>
#include <map>

using namespace std;

Behavior::Behavior(): state_(State::kStart) {
}

Behavior::~Behavior() {
}

Lane Behavior::SelectBestLane(Lane current_lane, std::list<Vehicle> &vehicles) {
  return current_lane;
}

std::pair<State,Lane> Behavior::Update(Map &map, Localization &localization, std::list<Vehicle> &vehicles) {

  Lane current_lane = map.GetLane(localization.d);

  switch(state_) {
    case State::kStart:
      state_ = State::kKeepLane;
      target_lane_ = current_lane;
      break;

    case State::kKeepLane:
      target_lane_ = SelectBestLane(current_lane, vehicles);
      if (target_lane_ != current_lane) {
        state_ = State::kPrepareChangeLane;
      }
      break;

    case State::kPrepareChangeLane:
      state_ = State::kChangeLane;
      break;

    case State::kChangeLane:
      if (current_lane == target_lane_) {
        state_ = State::kKeepLane;
      }
      break;
  }

  return make_pair(state_, target_lane_);
}
