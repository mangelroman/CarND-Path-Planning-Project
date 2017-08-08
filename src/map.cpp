#include "map.h"

#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <functional>

#include "spline.h"

using namespace std;

Map::Map(const string &map_file) {
  ifstream ifs(map_file, ifstream::in);

  double x;
  double y;
  double s;
  double dx;
  double dy;

  vector<double> x_wp;
  vector<double> y_wp;
  vector<double> s_wp;
  vector<double> dx_wp;
  vector<double> dy_wp;

  string line;
  while (getline(ifs, line)) {
    istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    x_wp.push_back(x);
    y_wp.push_back(y);
    s_wp.push_back(s);
    dx_wp.push_back(dx);
    dy_wp.push_back(dy);
  }
  ifs.close();

  // Add padding samples at the end of the road to compute trajectories
  // at the end of the road without bothering about s discontinuity
  for(int i = 0; i < kNumberOfPaddingSamples; i++) {
    x_wp.push_back(x_wp[i]);
    y_wp.push_back(y_wp[i]);
    s_wp.push_back(s_wp[i] + kRoadLength);
    dx_wp.push_back(dx_wp[i]);
    dy_wp.push_back(dy_wp[i]);
  }

  s_x_.set_points(s_wp, x_wp);
  s_y_.set_points(s_wp, y_wp);
  s_dx_.set_points(s_wp, dx_wp);
  s_dy_.set_points(s_wp, dy_wp);
}

Map::~Map() {}

pair<double, double> Map::FrenetToCartesian(pair<double,double> sd) const {

  double s = get<0>(sd);
  double d = get<1>(sd);

  double x = s_x_(s) + d * s_dx_(s);
  double y = s_y_(s) + d * s_dy_(s);

  return make_pair(x, y);
}

Lane Map::GetLane(double d) const {
  if (d < 0) {
    return Lane::kOutLeft;
  }

  if (d < kLaneWidth) {
    return Lane::kLeft;
  }

  if (d < 2 * kLaneWidth) {
    return Lane::kCenter;
  }

  if (d < 3 * kLaneWidth) {
    return Lane::kRight;
  }

  return Lane::kOutRight;
}

double Map::GetCenterOfLane(Lane lane) const {
  return (double(lane) + 0.5) * kLaneWidth;
}
