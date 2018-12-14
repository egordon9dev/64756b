#ifndef PID_H
#define PID_H
#include <stdbool.h>
#include "Point.hpp"

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
    int xAxisDir, rotationDir;

   public:
    Odometry_t(double L);
    void update();
    double getX();
    double getY();
    double getA();
    void setA(double a);
    void setX(double x);
    void setXAxisDir(int n);
    void setRotationDir(int n);
    void setY(double y);
    Point getPos();
};

void pidDriveInit(const Point& target, const int wait);
bool pidDrive();
bool pidTurn(const double angle, const int wait);
bool pidTurnSweep(double tL, double tR, int wait);
void pidDriveArcInit(Point start, Point target, double rMag, int rotationDirection, int wait);
void pidFollowArcInit(Point start, Point target, double rMag, int rotationDirection, int wait);
bool pidDriveArc();
void pidDriveArcBias(int b);
void printArcData();

extern Pid_t flywheelPid, clawPid, drfbPid, DLPid, DRPid, DLTurnPid, DRTurnPid, drivePid, turnPid, curvePid;
extern Slew_t flywheelSlew, drfbSlew, DLSlew, DRSlew, clawSlew;
extern Odometry_t odometry;
extern int g_pidTurnLimit;

#endif