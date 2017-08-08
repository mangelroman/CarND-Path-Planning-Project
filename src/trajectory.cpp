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

vector<double> Trajectory::JMT(vector<double> start, vector<double> end, double t) {

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
  Localization &localization,
  TrajectoryXY &previous_trajectory,
  FrenetPoint &previous_coordinates,
  BehaviorInfo &behavior)
{
  State state = behavior.state;
  double target_d = map.GetCenterOfLane(behavior.target_lane);
  double target_speed = behavior.target_speed;
  int consumed_points = kNumberOfPoints - previous_trajectory.first.size();

  switch(state) {
    case State::kStop:
      break;

    case State::kStart: {
        double s = localization.s;
        double d = localization.d;
        double v = localization.v;

        double dd = (target_d - d) / kNumberOfPoints;

        for (int i = 0; i < kNumberOfPoints; i++) {

          s += v * kTimeBetweenPoints;

          if (v < target_speed) {
            // Apply Constant acceleration
            s += kMaxLongitudinalAcceleration * pow(kTimeBetweenPoints, 2) / 2;
            v += kMaxLongitudinalAcceleration * kTimeBetweenPoints;
            v = min(v, target_speed);
          }

          d += dd;

          sd_points_[i] = make_pair(s, d);
        }
      }
      break;

    case State::kPrepareChangeLane: {

        auto sd = sd_points_[consumed_points];

        vector<double> s_start = {
          sd.first, //localization.s,
          localization.v,
          0
        };
        vector<double> s_end = {
          sd.first /*localization.s*/ + (localization.v + target_speed) * (kNumberOfPoints * kTimeBetweenPoints) / 2,
          target_speed,
          0
        };
        vector<double> d_start = {
          sd.second, //localization.d,
          0, //localization.d - sd_points_[consumed_points - 1].second,
          0
        };
        vector<double> d_end = {
          target_d,
          0,
          0
        };

        // cout << s_start[0] << "," << s_start[1] << "," << s_start[2] << "," << endl;
        // cout << s_end[0] << "," << s_end[1] << "," << s_end[2] << "," << endl;
        // cout << d_start[0] << "," << d_start[1] << "," << d_start[2] << "," << endl;
        // cout << d_end[0] << "," << d_end[1] << "," << d_end[2] << "," << endl;

        auto s_coeffs = JMT(s_start, s_end, kNumberOfPoints * kTimeBetweenPoints);
        auto d_coeffs = JMT(d_start, d_end, kNumberOfPoints * kTimeBetweenPoints);

        for (int i = 0; i < kNumberOfPoints; i++) {
          double s = PolyEval(s_coeffs, i * kTimeBetweenPoints);
          double d = PolyEval(d_coeffs, i * kTimeBetweenPoints);
          sd_points_[i] = make_pair(s, d);
        }

      }
      break;

    case State::kKeepLane:
    case State::kChangeLane: {

        auto sd1 = sd_points_[-1];
        auto sd2 = sd_points_[-2];

        double s = sd1.first;
        double d = sd1.second;
        double v = (sd1.first - sd2.first) / kTimeBetweenPoints;
        double dd = (target_d - sd1.second) / consumed_points;

        cout << s << "-" << d << "-" << v << "-" << dd << "-" << consumed_points << endl;

        for (int i = 0; i < consumed_points; i++) {

          s += v * kTimeBetweenPoints;

          if (v < target_speed) {
            // Apply Constant acceleration
            s += kMaxLongitudinalAcceleration * pow(kTimeBetweenPoints, 2) / 2;
            v += kMaxLongitudinalAcceleration * kTimeBetweenPoints;
            v = min(v, target_speed);
          }

          d += dd;

          sd_points_.append(make_pair(s, d));
        }
      }
      break;

  }

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  for(int i = 0; i < kNumberOfPoints; i++) {
    auto xy = map.FrenetToCartesian(sd_points_[i]);
    next_x_vals.push_back(xy.first);
    next_y_vals.push_back(xy.second);
  }

  return make_pair(next_x_vals, next_y_vals);
}
