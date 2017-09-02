#include "planner.h"

#include <cmath>
#include <iostream>

#include "spline.h"

using namespace std;

Planner::Planner(const std::string &map_file): map_(map_file), tick_(0) {
}

Planner::~Planner() {
}

void Planner::Reset() {
  prediction_.Reset();
  behavior_.Reset();
  trajectory_.Reset();
  tick_ = 0;
}


TrajectoryXY Planner::Update(Localization &loc,
                     SensorFusion &sf_data,
                     TrajectoryXY &previous_trajectory,
                     FrenetPoint &previous_coordinates)
{
  if (tick_++ < 50) {
    return previous_trajectory;
  }
  tick_ = 0;

  auto vehicles = prediction_.Update(map_, loc, sf_data);
  BehaviorInfo behavior_info = behavior_.Update(map_, loc, vehicles);

  cout << "STATE= " << int(behavior_info.state);
  cout << "  TARGET_LANE= " << int(behavior_info.target_lane);
  cout << "  TARGET_SPEED=" << behavior_info.target_speed;
  cout << "  LOCATION=" << loc.s << "/" << loc.d << "/" << loc.v << endl;

  return trajectory_.Generate(map_, loc, previous_trajectory, previous_coordinates, behavior_info);
}
