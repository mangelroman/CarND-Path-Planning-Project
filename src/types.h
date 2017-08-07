#ifndef TYPES_H
#define TYPES_H

#include <vector>

typedef std::pair<std::vector<double>,std::vector<double>> TrajectoryXY;

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

enum class Prediction {
  kStraight,
  kChangeLeft,
  kChangeRight
};

enum class Lane {
  kOutLeft = -1,
  kLeft = 0,
  kCenter = 1,
  kRight = 2,
  kOutRight = 3
};

enum class State {
  kStart,
  kKeepLane,
  kPrepareChangeLane,
  kChangeLane,
};

#endif // TYPES_H
