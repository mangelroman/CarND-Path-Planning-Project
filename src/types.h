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
  kStop = 0,
  kStart,
  kKeepLane,
  kChangeLane,
  kChangeSpeed
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

typedef std::vector<std::vector<double>> SensorFusion;

typedef struct {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double distance;
  double speed;
  Lane lane;
} VehicleInfo;

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

template<class T, size_t N> class CircularBuffer {
private:
  T items_[N];
  int front_;

public:
  CircularBuffer(): front_(0) { }
  ~CircularBuffer() { }

  void append(T value) {
      items_[front_] = value;
      front_ = (front_ + 1) % N;
  }

  T& operator [] (int index) {
      if (index < 0) {
        index += N;
      }
      int loc = (front_ + index) % N;
      return items_[loc];
  }
};

#endif // TYPES_H
