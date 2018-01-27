#include "PID.h"

PID::PID (float _kp, float _ki, float _kd, float *_setpoint) {
  kp = _kp;
  ki = _ki;
  kd = _kd;
  setpoint = _setpoint;
}


float PID::calc(float _angle) {

  float _diff_p = _angle - *setpoint;
  float _diff_d = last_d - _angle;
  float _return;

  sum_i += ki * _diff_p; //Blödsinn
  gg

  _return += kp * _diff_p;
  _return += sum_i;
  _return += kd * _diff_d;

  Serial.println(sum_i);
  _return = max (_return, limit_min);
  _return = min (_return, limit_max);

  last_d = _angle;

  return _return;
}


float PID::get_P() {
  return kp;
}
float PID::get_I() {
  return ki;
}
float PID::get_D() {
  return kd;
}

void PID::set_pid(float _p, float _i, float _d) {
  set_P(_p);
  set_I(_i);
  set_D(_d);
}

void PID::set_P(float _p) {
  kp = _p;
}
void PID::set_I(float _i) {
  ki = _i;
}
void PID::set_D(float _d) {
  kd = _d;
}
void PID::set_limits(float _min, float _max) {
  limit_min = _min;
  limit_max = _max;
}

