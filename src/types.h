#ifndef TYPES_H
#define TYPES_H

typedef std::pair<std::vector<double>,std::vector<double>> Trajectory;

typedef struct {
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
  STRAIGHT = 0,
  CHANGE_LEFT,
  CHANGE_RIGHT
};

enum class Lane {
  OutLeft = 0,
  Left,
  Center,
  Right,
  OutRight
};

#endif
