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
  State state = behavior.state;
  double target_d = map.GetCenterOfLane(behavior.target_lane);
  double target_speed = behavior.target_speed;
  int consumed_points = kNumberOfPoints - previous_trajectory.x.size();
  if (loc.s < sd_points_[0].s) {
    cout << "NEW LAP!!!" << endl;
    for (int i = 0; i < kNumberOfPoints; i++) {
      sd_points_[i].s -= map.GetRoadLength();
    }
  }

  switch(state) {

    case State::kStop:
      break;

    case State::kStart: {
        vector<double> v_start = {loc.v, 0, 0};
        vector<double> v_end = {target_speed, kAvgAcceleration, 0};
        vector<double> d_start = {loc.d, 0, 0};
        vector<double> d_end = {target_d, 0, 0};

        auto v_coeffs = JMT(v_start, v_end, kNumberOfPoints * kTimeBetweenPoints);
        auto d_coeffs = JMT(d_start, d_end, kNumberOfPoints * kTimeBetweenPoints);

        FrenetPoint point;
        point.s = loc.s;
        point.d = loc.d;

        for (int i = 0; i < kNumberOfPoints; i++) {
          double next_s = point.s + PolyEval(v_coeffs, i * kTimeBetweenPoints) * kTimeBetweenPoints;
          double curv = map.Curvature(point.s, next_s);

          next_s += point.d * sin(curv);

          point.s = next_s;
          point.d = PolyEval(d_coeffs, i * kTimeBetweenPoints);

          sd_points_[i] = point;
        }
      }
      break;

    case State::kChangingLanes:
    case State::kKeepLane: {

        FrenetPoint point = sd_points_[-1];
        for (int i = 0; i < consumed_points; i++) {

          double next_s = point.s + target_speed * kTimeBetweenPoints;
          double curv = map.Curvature(point.s, next_s);

          next_s += point.d * sin(curv);

          point.s = next_s;
          point.d = target_d;
          sd_points_.append(point);
        }
      }
      break;

    case State::kChangeSpeed: {

        FrenetPoint point;
        int acc_points = int(ceil(abs(target_speed - loc.v) / (kAvgAcceleration * kTimeBetweenPoints)));

        vector<double> v_start = {loc.v, 0, 0};
        vector<double> v_end = {target_speed, 0, 0};
        auto v_coeffs = JMT(v_start, v_end, acc_points * kTimeBetweenPoints);

        point = sd_points_[consumed_points];

        sd_points_[0] = point;
        point.d = target_d;

        for (int i = 1; i < kNumberOfPoints; i++) {
          double next_s;
          if (i < acc_points) {
            next_s = point.s + PolyEval(v_coeffs, i * kTimeBetweenPoints) * kTimeBetweenPoints;
          }
          else {
            next_s = point.s + target_speed * kTimeBetweenPoints;
          }

          double curvature = map.Curvature(point.s, next_s);
          next_s += point.d * sin(curvature);

          point.s = next_s;
          sd_points_[i] = point;
        }
      }
      break;

    case State::kChangeLane: {

        FrenetPoint point;
        point = sd_points_[consumed_points];

        int acc_points = int(ceil(abs(target_speed - loc.v) / (kAvgAcceleration * kTimeBetweenPoints)));
        double vd = (point.d - sd_points_[consumed_points - 1].d) / kTimeBetweenPoints;

        vector<double> v_start = {loc.v, 0, 0};
        vector<double> v_end = {target_speed, 0, 0};
        vector<double> d_start = { point.d, vd, 0 };
        vector<double> d_end = { target_d, 0, 0 };
        auto v_coeffs = JMT(v_start, v_end, acc_points * kTimeBetweenPoints);
        auto d_coeffs = JMT(d_start, d_end, kPointsToChangeLane * kTimeBetweenPoints);


        sd_points_[0] = point;

        for (int i = 1; i < kNumberOfPoints; i++) {
          double next_s;
          if (i < acc_points) {
            next_s = point.s + PolyEval(v_coeffs, i * kTimeBetweenPoints) * kTimeBetweenPoints;
          }
          else {
            next_s = point.s + target_speed * kTimeBetweenPoints;
          }

          if (i < kPointsToChangeLane) {
            point.d = PolyEval(d_coeffs, i * kTimeBetweenPoints);
          }
          else {
            point.d = target_d;
          }

          double curvature = map.Curvature(point.s, next_s);
          next_s += point.d * sin(curvature);

          point.s = next_s;
          sd_points_[i] = point;
        }
      }
      break;
  }

  TrajectoryXY next_trajectory;
  for(int i = 0; i < kNumberOfPoints; i++) {
    XYPoint point = map.FrenetToCartesian(sd_points_[i]);
    next_trajectory.x.push_back(point.x);
    next_trajectory.y.push_back(point.y);
  }

  return next_trajectory;
}
