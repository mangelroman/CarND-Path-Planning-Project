#include "map.h"

#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <functional>

#include "spline.h"

Map::Map(const std::string &map_file) {
  std::ifstream ifs(map_file, std::ifstream::in);

  double x;
  double y;
  double s;
  double dx;
  double dy;

  std::string line;
  while (getline(ifs, line)) {
    std::istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    x_.push_back(x);
    y_.push_back(y);
    s_.push_back(s);
    dx_.push_back(dx);
    dy_.push_back(dy);
  }

  ifs.close();
}

Map::~Map() {}

std::pair<double, double> Map::FrenetToCartesian(double s, double d) const {

  if ( s > ROAD_LENGTH) { s -= ROAD_LENGTH; }

  // index of the next way point
  int closest_waypoint = std::distance(s_.begin(), std::lower_bound(s_.begin(), s_.end(), s));
  /*
  // Conversion using interpolation for better accuracy
  std::vector<double> local_s;
  std::vector<double> local_x;
  std::vector<double> local_y;
  std::vector<double> local_dx;
  std::vector<double> local_dy;

  for ( int i = -5; i < 6; ++i ) {
    int index = closest_waypoint + i;
    if ( index < 0 ) {
      index += s_.size();
      local_s.push_back(s_[index] - ROAD_LENGTH);
    } else if ( index >= s_.size() ) {
      index -= s_.size();
      local_s.push_back(s_[index] + ROAD_LENGTH);
    } else {
      local_s.push_back(s_[index]);
    }
    local_x.push_back(x_[index]);
    local_y.push_back(y_[index]);
    local_dx.push_back(dx_[index]);
    local_dy.push_back(dy_[index]);
  }

  tk::spline spline_sx, spline_sy, spline_sdx, spline_sdy;
  spline_sx.set_points(local_s, local_x);
  spline_sy.set_points(local_s, local_y);
  spline_sdx.set_points(local_s, local_dx);
  spline_sdy.set_points(local_s, local_dy);

  double x = spline_sx(s);
  double y = spline_sy(s);
  double dx = spline_sdx(s);
  double dy = spline_sdy(s);

  x += d * dx;
  y += d * dy;
  */

  int wp2 = closest_waypoint;
  int wp1 = (wp2 > 0) ? (wp2 - 1) : 0;

  double heading = atan2((y_[wp2] - y_[wp1]), (x_[wp2] - x_[wp1]));
  // the x,y,s along the segment
  double seg_s = (s - s_[wp1]);

  double seg_x = x_[wp1] + seg_s * cos(heading);
  double seg_y = y_[wp1] + seg_s * sin(heading);

  double perp_heading = heading - M_PI/2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return std::make_pair(x, y);
}
