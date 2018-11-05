#ifndef PID_H
#define PID_H
#include <stdbool.h>

class Slew_t {
   public:
    double slewRate, output;
    long prevTime;
    Slew_t();
    double update(double in);
};
class Pid_t {
   public:
    double unwind, DONE_ZONE, maxIntegral, iActiveZone, target, sensVal, prevSensVal, prevErr, errTot, kp, ki, kd, deriv;
    long prevTime, doneTime, prevDUpdateTime;
    Pid_t();
    double update();
};

extern Pid_t flywheelPid, clawPid;
extern Slew_t flywheelSlew, drfbSlew;
#define LONG_MAX 2147483647
#define DBL_MAX 999999999.999999

#endif