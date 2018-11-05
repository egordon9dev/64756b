#include "pid.hpp"
#include "main.h"
Pid_t flywheelPid, clawPid;
Slew_t flywheelSlew, drfbSlew;
Slew_t::Slew_t() {
    slewRate = 100.0;
    output = 0;
    prevTime = pros::millis();
}
Pid_t::Pid_t() {
    doneTime = LONG_MAX;
    DONE_ZONE = 10;
    maxIntegral = DBL_MAX;
    iActiveZone = target = prevSensVal = sensVal = prevErr = errTot = unwind = deriv = kp = ki = kd = 0.0;
    prevTime = prevDUpdateTime = 0;
}

/*
  in: input voltage
*/
double Slew_t::update(double in) {
    long dt = pros::millis() - prevTime;
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
    long dt = pros::millis() - prevTime;
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