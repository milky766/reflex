#include "Angle_Pid.h"

PidController::AnglePidController(const double &kp, const double &ki, const double &kd,
                             const double &upper_clamp, const double &lower_clamp) {
  param_.kp = kp;
  param_.ki = ki;
  param_.kd = kd;

  lower_clamp_ = lower_clamp;
  upper_clamp_ = upper_clamp;

  state_.ep = state_.ei = state_.ed = 0.0;

  state_.t = micros() * 1e-6;

  output_ = 0.0;
}

double AnglePidController::update(const double &e) {
  double t = micros() * 1e-6;
  double dt = t - state_.t;
  state_.t = t;

  state_.ed = (e - state_.ep) / dt;
  state_.ei += e * dt;
  state_.ep = e;

  output_ = param_.kp * state_.ep + param_.ki * state_.ei + param_.kd * state_.ed;

  if(output_ < lower_clamp_)
  {
    return lower_clamp_;
  }

  if(output_ > upper_clamp_)
  {
    return upper_clamp_;
  }

  return output_;
}

uint64_t AnglePidController::micros()
{
  uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                                                                      now().time_since_epoch()).count();
  return us;
}
