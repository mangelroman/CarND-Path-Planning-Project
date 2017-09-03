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
  // without bothering about s discontinuity
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

XYPoint Map::FrenetToCartesian(FrenetPoint frenet) const {

  XYPoint result;
  while (frenet.s >= kRoadLength) frenet.s -= kRoadLength;

  result.x = s_x_(frenet.s) + frenet.d * s_dx_(frenet.s);
  result.y = s_y_(frenet.s) + frenet.d * s_dy_(frenet.s);

  return result;
}

double Map::Distance(double s1, double s2) const {
  double d = s2 - s1;
  if (d > kRoadLength / 2) {
    d -= kRoadLength;
  }
  else if (d < -kRoadLength / 2) {
    d += kRoadLength;
  }
  return d;
}

double Map::Curvature(double s1, double s2) const {

	double d_heading_diff = atan2(s_dy_(s1), s_dx_(s1)) - atan2(s_dy_(s2), s_dx_(s2));
  if (d_heading_diff < -M_PI) {
      d_heading_diff += 2 * M_PI;
  }
  else if (d_heading_diff > M_PI) {
    d_heading_diff -= 2 * M_PI;
  }
  return d_heading_diff;
}

double Map::TangentialVelocity(double s, double vx, double vy) const {
  // The projection of v on d rotated -90 degrees
  double dx = s_dx_(s);
  double dy = s_dy_(s);

  return vx * -dy + vy * dx;
}

double Map::NormalVelocity(double s, double vx, double vy) const {
  // The projection of v on d
  double dx = s_dx_(s);
  double dy = s_dy_(s);

  return vx * dx + vy * dy;
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
  double d = (double(lane) + 0.5) * kLaneWidth;
  if (lane == Lane::kRight) {
    d -= kOuterRoadGuard;
  }
  if (lane == Lane::kLeft) {
    d += kOuterRoadGuard;
  }
  return d;
}
