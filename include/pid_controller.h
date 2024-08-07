#ifndef PID_CONTROLLER_h
#define PID_CONTROLLER_h

#include <chrono>

class PidController {
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

  PidController(const double &kp, const double &ki, const double &kd,
                const double &upper_clamp, const double &lower_clamp);

  ~PidController() = default;

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
