#ifndef PID_H
#define PID_H
#include <stdbool.h>

class Slew_t {
   public:
    double slewRate, output;
    int prevTime;
    Slew_t();
    double update(double in);
};
class Pid_t {
   public:
    double unwind, DONE_ZONE, maxIntegral, iActiveZone, target, sensVal, prevSensVal, prevErr, errTot, kp, ki, kd, deriv;
    int prevTime, doneTime, prevDUpdateTime;
    Pid_t();
    double update();
};

extern Pid_t flywheelPid, clawPid, drfbPid;
extern Slew_t flywheelSlew, drfbSlew;

#endif