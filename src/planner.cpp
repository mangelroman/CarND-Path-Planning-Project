#include "planner.h"

#include <cmath>
#include <string>

#include "spline.h"

using namespace std;

Planner::Planner(const std::string &map_file): map_(map_file) {
}

Planner::~Planner() {
}

void Planner::Update(Localization &localization,
                     std::vector<std::vector<double>> &sf_data,
                     Trajectory &previous_trajectory)
{
  localization_ = localization;
}

Trajectory Planner::Plan() {

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double dist_inc = 0.4;
  for(int i = 0; i < 50; i++) {
    auto xy = map_.FrenetToCartesian(localization_.s + dist_inc * i, localization_.d);
    next_x_vals.push_back(get<0>(xy));
    next_y_vals.push_back(get<1>(xy));
  }

  return make_pair(next_x_vals, next_y_vals);
}
