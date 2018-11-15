#include "pid.hpp"
#include "main.h"
#include "setup.hpp"

Pid_t flywheelPid, clawPid, drfbPid;
Slew_t flywheelSlew, drfbSlew, DLSlew, DRSlew;
Odometry_t odometry(6.982698);
Slew_t::Slew_t() {
    slewRate = 100.0;
    output = 0;
    prevTime = pros::millis();
}
Pid_t::Pid_t() {
    doneTime = INT_MAX;
    DONE_ZONE = 10;
    maxIntegral = 9999999;
    dInactiveZone = iActiveZone = target = prevSensVal = sensVal = prevErr = errTot = unwind = deriv = kp = ki = kd = 0.0;
    prevTime = prevDUpdateTime = 0;
}
Odometry_t::Odometry_t(double L) {
    this->L = L;
    this->a = PI / 2;
    this->x = this->y = this->prevDL = this->prevDR = 0.0;
}
double Odometry_t::getX() { return x; }
double Odometry_t::getY() { return y; }
double Odometry_t::getA() { return a; }
void Odometry_t::update() {
    double curDL = getDL(), curDR = getDR();
    double deltaDL = (curDL - prevDL) / ticksPerInch, deltaDR = (curDR - prevDR) / ticksPerInch;
    double deltaDC = (deltaDL + deltaDR) / 2.0;
    double deltaA = (deltaDR - deltaDL) / (2.0 * L);
    x += deltaDC * cos(a + deltaA / 2);
    y += deltaDC * sin(a + deltaA / 2);
    a += deltaA;
    prevDL = curDL;
    prevDR = curDR;
}

/*
  in: input voltage
*/
double Slew_t::update(double in) {
    int dt = pros::millis() - prevTime;
    if (dt > 1000) dt = 0;
    prevTime = pros::millis();
    double maxIncrease = slewRate * dt;
    double outputRate = (double)(in - output) / (double)dt;
    if (fabs(outputRate) < slewRate) {
        output = in;
    } else if (outputRate > 0) {
        output += maxIncrease;
    } else {
        output -= maxIncrease;
    }
    return output;
}
// proportional + integral + derivative control feedback
double Pid_t::update() {
    int dt = pros::millis() - prevTime;
    if (dt > 1000) dt = 0;
    prevTime = pros::millis();
    // PROPORTIONAL
    double err = target - sensVal;
    double p = err * kp;
    // DERIVATIVE
    double d = deriv;  // set d to old derivative
    double derivativeDt = pros::millis() - prevDUpdateTime;
    if (derivativeDt > 1000) {
        prevSensVal = sensVal;
        prevDUpdateTime = pros::millis();
    } else if (derivativeDt >= 15) {
        d = ((prevSensVal - sensVal) * kd) / derivativeDt;
        prevDUpdateTime = pros::millis();
        deriv = d;  // save new derivative
        prevSensVal = sensVal;
    }
    if (fabs(err) < dInactiveZone) d = 0;
    // INTEGRAL
    errTot += err * dt;
    if (fabs(err) > iActiveZone) errTot = 0;
    if (fabs(d) > 10) {
        double maxErrTot = maxIntegral / ki;
        if (errTot > maxErrTot) errTot = maxErrTot;
        if (errTot < -maxErrTot) errTot = -maxErrTot;
    }
    if ((err > 0.0 && errTot < 0.0) || (err < 0.0 && errTot > 0.0) || abs(err) < 0.001) {
        if (fabs(err) - unwind > -0.001) {
            errTot = 0.0;
            // printf("UNWIND\n");
        }
    }
    if (fabs(unwind) < 0.001 && fabs(err) < 0.001) {
        errTot = 0.0;
        // printf("UNWIND\n");
    }
    double i = errTot * ki;
    // done zone
    if (fabs(err) <= DONE_ZONE && doneTime > pros::millis()) {
        doneTime = pros::millis();
        printf("DONE\n");
    }
    // derivative action: slowing down
    /*if (fabs(d) > (fabs(p)) * 20.0) {
          errTot = 0.0;
      }*/
    prevErr = err;
    // printf("p: %lf, i: %lf, d: %lf\t", p, i, d);
    // OUTPUT
    return p + i + d;
}