#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include <iostream>
#include <math.h>

class AnglePidController {
public:
  struct Param {
    double kp;
    double ki;
    double kd;
  };

  struct State {
    double ep;
    double ei;
    double ed;
    double t;
  };

  AnglePidController(const double &kp, const double &ki, const double &kd,
                const double &upper_clamp, const double &lower_clamp);

  ~AnglePidController() = default;

  double update(const double &e);

  uint64_t micros();

private:
  State state_{};
  Param param_{};
  double lower_clamp_;
  double upper_clamp_;
  double output_;
};

#endif