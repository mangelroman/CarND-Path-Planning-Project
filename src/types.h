#ifndef TYPES_H
#define TYPES_H

#include <vector>

enum class Lane {
  kOutLeft = -1,
  kLeft = 0,
  kCenter = 1,
  kRight = 2,
  kOutRight = 3
};

enum class State {
  kStart = 0,
  kChangeSpeed,
  kKeepLane,
  kChangeLane,
  kEmergencyBreak,
  kStop,
};

typedef struct {
  std::vector<double> x;
  std::vector<double> y;
} TrajectoryXY;

typedef struct {
  double s;
  double d;
} FrenetPoint;

typedef struct {
  double x;
  double y;
} XYPoint;

typedef struct {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
} SensorFusion;

typedef struct {
  double x;
  double y;
  double v;
  double yaw;
  double s;
  double d;
} Localization;

typedef struct {
  State state;
  Lane target_lane;
  double target_speed;
} BehaviorInfo;

#endif // TYPES_H
