#ifndef MAP_H
#define MAP_H

#include <vector>

#include "types.h"

class Map {

private:

  const size_t NUMBER_OF_LANES = 3;
  const double LANE_WIDTH = 4.0;
  const double ROAD_LENGTH = 6945.554;

  // Coordinates of the way points
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> s_;
  std::vector<double> dx_;
  std::vector<double> dy_;

public:

  Map(const std::string &filename);
  ~Map();

  std::pair<double, double> FrenetToCartesian(double s, double d) const;
  //std::pair<double, double> cartesianToFrenet(double x, double y) const;

  double GetRoadLength() const { return ROAD_LENGTH; }
  double GetLaneWidth() const { return LANE_WIDTH; }
  size_t GetNumberOfLanes() const { return NUMBER_OF_LANES; }

};

#endif //MAP_H
