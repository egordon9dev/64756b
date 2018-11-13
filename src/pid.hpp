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
    double unwind, DONE_ZONE, maxIntegral, iActiveZone, dInactiveZone, target, sensVal, prevSensVal, prevErr, errTot, kp, ki, kd, deriv;
    int prevTime, doneTime, prevDUpdateTime;
    Pid_t();
    double update();
};
class Odometry_t {
   private:
    double x, y, a, L, prevDL, prevDR;

   public:
    Odometry_t(double L);
    void update();
    double getX();
    double getY();
    double getA();
};

extern Pid_t flywheelPid, clawPid, drfbPid;
extern Slew_t flywheelSlew, drfbSlew;
extern Odometry_t odometry;

#endif