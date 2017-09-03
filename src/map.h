#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>

#include "types.h"
#include "spline.h"

class Map {

private:

  const size_t kNumberOfLanes = 3;
  const double kLaneWidth = 4.0;
  const double kRoadLength = 6945.554;
  const double kOuterRoadGuard = 0.15;
  const int    kNumberOfPaddingSamples = 1;

  tk::spline s_x_;
  tk::spline s_y_;
  tk::spline s_dx_;
  tk::spline s_dy_;

public:

  Map(const std::string &filename);
  ~Map();

  XYPoint FrenetToCartesian(FrenetPoint frenet) const;
  double Distance(double s1, double s2) const;
  double Curvature(double s1, double s2) const;
  double TangentialVelocity(double s, double vx, double vy) const;
  double NormalVelocity(double s, double vx, double vy) const;

  Lane GetLane(double d) const;
  double GetCenterOfLane(Lane lane) const;

  double GetRoadLength() const { return kRoadLength; }
  double GetLaneWidth() const { return kLaneWidth; }
  size_t GetNumberOfLanes() const { return kNumberOfLanes; }

};

#endif //MAP_H
