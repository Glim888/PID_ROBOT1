#ifndef __PID__
#define __PID__

#include <Arduino.h>


class PID {

  private:

    float kp, ki, kd, *setpoint, last_d = 0, sum_i = 0, limit_min = -1000, limit_max = 1000;

  public:
    PID (float _kp, float _ki, float _kd, float *_setpoint);

    float calc(float _angle);

    float get_P();
    float get_I();
    float get_D();

    void set_pid(float _p, float _i, float _d);
    void set_P(float _p);
    void set_I(float _i);
    void set_D(float _d);
    void set_limits(float _min, float _max);

};
#endif
