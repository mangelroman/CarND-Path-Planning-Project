#include "prediction.h"

#include <iostream>
#include <map>
#include <cmath>

using namespace std;

Prediction::Prediction() {
}

Prediction::~Prediction() {
}

void Prediction::Reset() {

}

Lane Prediction::GetBestLane(Map &map, Localization &loc, std::vector<SensorFusion> &sensor_fusion) {
  Lane current_lane = map.GetLane(loc.d);

  std::map<Lane,double> cost;

  cost[Lane::kOutLeft] = 100000;
  cost[Lane::kLeft] = (current_lane == Lane::kRight) ? kLaneScoreFarLaneWeight : 0;
  cost[Lane::kCenter] = 0;
  cost[Lane::kRight] = (current_lane == Lane::kLeft) ? kLaneScoreFarLaneWeight : 0;;
  cost[Lane::kOutRight] = 100000;

  for (auto car : sensor_fusion) {
    Lane car_lane = map.GetLane(car.d);
    double gap = map.ComputeDistance(loc.s, car.s);
    double car_speed = sqrt(pow(car.vx, 2) + pow(car.vy, 2));
    double speed_diff = loc.v - car_speed;
    int lane_diff = abs(int(current_lane) - int(car_lane));

    if ((gap < kVisibleFrontDistance) && (gap > -kVisibleBackDistance)) {
       // Object is close by
      if (gap > 0) {
        cost[car_lane] += kLaneScoreFrontGapWeight / gap;
        cost[car_lane] += kLaneScoreSpeedWeight * speed_diff;
      }
      else {
        if (lane_diff > 0) {
          // Car is not behind our lane
          cost[car_lane] += kLaneScoreBackGapWeight / (-gap);
          cost[car_lane] += kLaneScoreSpeedWeight * (-speed_diff);
          cost[car_lane] += kChangeLaneCostMargin;
        }
      }
    }
  }
  /*
  cout << "COSTS=(";
  for (auto c : cost) {
    cout << "L" << int(c.first) << ":" << c.second << ",";
  }
  cout << ")" << endl;
  */
  auto best_lane = min_element(cost.begin(), cost.end(), [] (const pair<Lane,double> & p1, const pair<Lane,double> & p2) {
        return p1.second < p2.second;
  });

  return best_lane->first;
}
