#ifndef MAP_H
#define MAP_H

#include <vector>

#include "types.h"
#include "spline.h"

class Map {

private:

  const size_t kNumberOfLanes = 3;
  const double kLaneWidth = 4.0;
  const double kRoadLength = 6945.554;
  const int    kNumberOfPaddingSamples = 10;

  tk::spline s_x_;
  tk::spline s_y_;
  tk::spline s_dx_;
  tk::spline s_dy_;

public:

  Map(const std::string &filename);
  ~Map();

  std::pair<double, double> FrenetToCartesian(std::pair<double,double> sd) const;
  Lane GetLane(double d) const;
  double GetCenterOfLane(Lane lane) const;

  double GetRoadLength() const { return kNumberOfLanes; }
  double GetLaneWidth() const { return kLaneWidth; }
  size_t GetNumberOfLanes() const { return kRoadLength; }

};

#endif //MAP_H
