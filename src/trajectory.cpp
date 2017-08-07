#include "trajectory.h"

#include <cmath>

using namespace std;

Trajectory::Trajectory() {
}

Trajectory::~Trajectory() {
}

TrajectoryXY Trajectory::Generate(
  Map &map,
  Localization &localization,
  TrajectoryXY &previous_trajectory,
  std::pair<State,Lane> &behavior)
{

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double dist_inc = 0.4;
  for(int i = 0; i < 50; i++) {
    auto xy = map.FrenetToCartesian(localization.s + dist_inc * i, localization.d);
    next_x_vals.push_back(get<0>(xy));
    next_y_vals.push_back(get<1>(xy));
  }

  return make_pair(next_x_vals, next_y_vals);
}
