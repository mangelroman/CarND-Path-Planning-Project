#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <map>

#include "types.h"
#include "map.h"
#include "vehicle.h"

class Planner {

private:
  std::map<int,Vehicle> vehicles_;
  Map map_;
  Localization localization_;

public:

  Planner(const std::string &map_file);
  ~Planner();

  void Update(Localization &localization,
              std::vector<std::vector<double>> &sf_data,
              Trajectory &previous_trajectory);

  Trajectory Plan();
};

#endif // PLANNER_H
