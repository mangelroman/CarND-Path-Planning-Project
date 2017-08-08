#include "planner.h"

#include <cmath>
#include <string>

#include "spline.h"

using namespace std;

Planner::Planner(const std::string &map_file): map_(map_file), tick_(0) {
}

Planner::~Planner() {
}

void Planner::Reset() {
  vehicles_.clear();
  behavior_.Reset();
  trajectory_.Reset();
}


TrajectoryXY Planner::Update(Localization &localization,
                     std::vector<SensorFusion> &sensor_fusion,
                     TrajectoryXY &previous_trajectory,
                     FrenetPoint &previous_coordinates)
{
  if (tick_++ % 50) {
    return previous_trajectory;
  }

  // Update visible vehicles
  for (auto &vehicle: vehicles_) {
      vehicle.ClearVisible();
  }

  for(SensorFusion &sf : sensor_fusion) {

    bool found = false;
    for (auto &vehicle: vehicles_) {
      if (vehicle.GetId() == sf.id) {
        vehicle.Update(map_, sf);
        found = true;
        break;
      }
    }

    if (!found) {
      Vehicle vehicle(sf.id);
      vehicle.Update(map_, sf);
      vehicles_.push_back(vehicle);
    }
  }

  for (auto it = vehicles_.begin(); it != vehicles_.end(); it++) {
    if (it->IsVisible()) {
      it->Predict();
    }
    else {
      vehicles_.erase(it);
    }
  }


  auto state = behavior_.Update(map_, localization, vehicles_);
  return trajectory_.Generate(map_, localization, previous_trajectory, previous_coordinates, state);
}
