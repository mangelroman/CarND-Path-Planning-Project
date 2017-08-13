#include "trajectory.h"

#include <iostream>
#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "spline.h"

using namespace std;

Trajectory::Trajectory() {
}

Trajectory::~Trajectory() {
}

void Trajectory::Reset() {
}

vector<double> Trajectory::JMT(vector<double> &start, vector<double> &end, double t) {

  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  Eigen::MatrixXd A(3,3);
  A << t3,     t4,      t5,
       3 * t2, 4 * t3,  5 * t4,
       6 * t,  12 * t2, 20 * t3;

  Eigen::VectorXd B(3);
  B << end[0] - (start[0] + start[1] * t + start[2] * t2 / 2),
       end[1] - (start[1] + start[2] * t),
       end[2] - start[2];

  Eigen::VectorXd C(3);
  C = A.inverse() * B;

  return {start[0], start[1], start[2] / 2, C[0], C[1], C[2] };
}

double Trajectory::PolyEval(vector<double> &coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

TrajectoryXY Trajectory::Generate(
  Map &map,
  Localization &loc,
  TrajectoryXY &previous_trajectory,
  FrenetPoint &previous_coordinates,
  BehaviorInfo &behavior)
{
  if (loc.s < sd_points_[0].s) {
    cout << "NEW LAP!!! Loc= " << loc.s << " Path[0]=" << sd_points_[0].s << endl;
    for (int i = 0; i < kNumberOfPoints; i++) {
      sd_points_[i].s -= map.GetRoadLength();
    }
  }

  State state = behavior.state;
  double target_d = map.GetCenterOfLane(behavior.target_lane);
  double target_speed = behavior.target_speed;
  int consumed_points = kNumberOfPoints - previous_trajectory.x.size();

  switch(state) {

    case State::kStart:
      break;

    case State::kStop:
      break;

    case State::kEmergencyBreak: {
        FrenetPoint new_point;
        new_point.s = loc.s;
        new_point.d = loc.d;
        double v = loc.v;

        for (int i = 0; i < kNumberOfPoints; i++) {

          new_point.s += v * kTimeBetweenPoints;

          if (v > target_speed) {
            // Apply Constant acceleration
            new_point.s -= kMaxLDeceleration * pow(kTimeBetweenPoints, 2) / 2;
            v -= kMaxLAcceleration * kTimeBetweenPoints;
            v = max(v, target_speed);
          }

          sd_points_[i] = new_point;
          xy_points_[i] = map.FrenetToCartesian(new_point);
        }
      }

    case State::kChangeSpeed: {
        FrenetPoint start;
        double vd;
        if (consumed_points == kNumberOfPoints) {
          start.s = loc.s;
          start.d = loc.d;
          vd = 0;
        }
        else {
          start = sd_points_[consumed_points];
          vd = (start.d - sd_points_[consumed_points - 1].d) / kTimeBetweenPoints;
        }
        double a = (loc.v < target_speed) ? kMaxLAcceleration : -kMaxLAcceleration;
        double time_to_target_speed = (target_speed - loc.v) / a;

        vector<double> s_start = {
          start.s,
          loc.v,
          0
        };
        vector<double> s_end = {
          start.s + loc.v * time_to_target_speed + a * pow(time_to_target_speed, 2) / 2,
          target_speed,
          0
        };
        vector<double> d_start = {
          start.d,
          vd,
          0
        };
        vector<double> d_end = {
          target_d,
          0,
          0
        };

        auto s_coeffs = JMT(s_start, s_end, time_to_target_speed);
        auto d_coeffs = JMT(d_start, d_end, time_to_target_speed);

        int points_to_target_speed = int(ceil(time_to_target_speed / kTimeBetweenPoints));

        FrenetPoint new_point = start;

        for (int i = 0; i < kNumberOfPoints; i++) {

          if (i < points_to_target_speed) {
            new_point.s = PolyEval(s_coeffs, i * kTimeBetweenPoints);
            new_point.d = PolyEval(d_coeffs, i * kTimeBetweenPoints);
          }
          else {
            new_point.s += target_speed * kTimeBetweenPoints;
            new_point.d = target_d;
          }
          sd_points_[i] = new_point;
          xy_points_[i] = map.FrenetToCartesian(new_point);
        }
      }
      break;


    case State::kChangeLane: {

        FrenetPoint start;
        double vd;
        if (consumed_points == kNumberOfPoints) {
          start.s = loc.s;
          start.d = loc.d;
          vd = 0;
        }
        else {
          start = sd_points_[consumed_points];
          vd = (start.d - sd_points_[consumed_points - 1].d) / kTimeBetweenPoints;
        }

        vector<double> s_start = {
          start.s,
          loc.v,
          0
        };
        vector<double> s_end = {
          start.s + (loc.v + target_speed) * (kNumberOfPoints * kTimeBetweenPoints) / 2,
          target_speed,
          0
        };
        vector<double> d_start = {
          start.d,
          vd,
          0
        };
        vector<double> d_end = {
          target_d,
          0,
          0
        };

        auto s_coeffs = JMT(s_start, s_end, kNumberOfPoints * kTimeBetweenPoints);
        auto d_coeffs = JMT(d_start, d_end, kNumberOfPoints * kTimeBetweenPoints);

        FrenetPoint new_point;
        for (int i = 0; i < kNumberOfPoints; i++) {
          new_point.s = PolyEval(s_coeffs, i * kTimeBetweenPoints);
          new_point.d = PolyEval(d_coeffs, i * kTimeBetweenPoints);
          sd_points_[i] = new_point;
          xy_points_[i] = map.FrenetToCartesian(new_point);
        }

      }
      break;

    case State::kKeepLane: {

        FrenetPoint new_point = sd_points_[-1];

        double dd = (target_d - new_point.d) / consumed_points;

        for (int i = 0; i < consumed_points; i++) {

          new_point.s += target_speed * kTimeBetweenPoints;
          new_point.d += dd;

          sd_points_.append(new_point);
          xy_points_.append(map.FrenetToCartesian(new_point));
        }
      }
      break;
  }

  TrajectoryXY next_trajectory;
  for(int i = 0; i < kNumberOfPoints; i++) {
    next_trajectory.x.push_back(xy_points_[i].x);
    next_trajectory.y.push_back(xy_points_[i].y);
  }

  return next_trajectory;
}
