#include "pid.hpp"
#include "main.h"
#include "setup.hpp"
using pros::millis;
using std::cout;
Pid_t flywheelPid, clawPid, drfbPid, DLPid, DRPid, DLTurnPid, DRTurnPid, drivePid, turnPid;
Slew_t flywheelSlew, drfbSlew, DLSlew, DRSlew;
Odometry_t odometry(6.982698);

Slew_t::Slew_t() {
    slewRate = 100.0;
    output = 0;
    prevTime = millis();
}
Pid_t::Pid_t() {
    doneTime = BIL;
    DONE_ZONE = 10;
    maxIntegral = 9999999;
    dInactiveZone = iActiveZone = target = prevSensVal = sensVal = prevErr = errTot = unwind = deriv = kp = ki = kd = 0.0;
    prevTime = prevDUpdateTime = 0;
}
Odometry_t::Odometry_t(double L) {
    this->L = L;
    this->a = PI / 2;
    this->x = this->y = this->prevDL = this->prevDR = 0.0;
    this->xAxisDir = this->rotationDir = 1;
}
double Odometry_t::getX() { return x; }
double Odometry_t::getY() { return y; }
double Odometry_t::getA() { return a; }
void Odometry_t::setA(double a) { this->a = a; }
void Odometry_t::setX(double x) { this->x = x; }
void Odometry_t::setY(double y) { this->y = y; }
void Odometry_t::setXAxisDir(int n) { xAxisDir = n < 0 ? -1 : 1; }
void Odometry_t::setRotationDir(int n) { rotationDir = n < 0 ? -1 : 1; }
Point Odometry_t::getPos() {
    Point p(x, y);
    return p;
}

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
    int dt = millis() - prevTime;
    if (dt > 1000) dt = 0;
    prevTime = millis();
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
    int dt = millis() - prevTime;
    if (dt > 1000) dt = 0;
    prevTime = millis();
    // PROPORTIONAL
    double err = target - sensVal;
    double p = err * kp;
    // DERIVATIVE
    double d = deriv;  // set d to old derivative
    double derivativeDt = millis() - prevDUpdateTime;
    if (derivativeDt > 1000) {
        prevSensVal = sensVal;
        prevDUpdateTime = millis();
    } else if (derivativeDt >= 15) {
        d = ((prevSensVal - sensVal) * kd) / derivativeDt;
        prevDUpdateTime = millis();
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
    if (fabs(err) <= DONE_ZONE && doneTime > millis()) {
        doneTime = millis();
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
Point g_target;
bool pidDrive(const Point& target, const int wait) {
    g_target = target;
    Point pos(odometry.getX(), odometry.getY());
    static int prevT = 0;
    static Point prevPos(0, 0);
    int dt = millis() - prevT;
    bool returnVal = false;
    if (dt < 100) {
        // Point dir = pos - prevPos;
        Point targetDir = target - pos;
        if (targetDir.mag() < 0.001) {
            setDL(0);
            setDR(0);
        } else {
            // error detection
            Point dirOrientation(cos(odometry.getA()), sin(odometry.getA()));
            double aErr = acos(clamp((dirOrientation * targetDir) / (dirOrientation.mag() * targetDir.mag()), -1.0, 1.0));

            // allow for driving backwards
            int driveDir = 1;
            if (aErr > PI / 2) {
                driveDir = -1;
                aErr = PI - aErr;
            }
            if (dirOrientation < targetDir) aErr *= -driveDir;
            if (dirOrientation > targetDir) aErr *= driveDir;

            // error correction
            double curA = odometry.getA();
            drivePid.target = 0.0;
            drivePid.sensVal = targetDir.mag() * cos(aErr);
            if (drivePid.sensVal < 4) aErr = 0;
            turnPid.target = 0;
            turnPid.sensVal = aErr;
            int turnPwr = clamp((int)turnPid.update(), -8000, 8000);
            int drivePwr = clamp((int)drivePid.update(), -8000, 8000);
            // prevent turn saturation
            // if (abs(turnPwr) > 0.2 * abs(drivePwr)) turnPwr = (turnPwr < 0 ? -1 : 1) * 0.2 * abs(drivePwr);
            setDL(-drivePwr * driveDir - turnPwr);
            setDR(-drivePwr * driveDir + turnPwr);
        }
        if (drivePid.doneTime + wait < millis()) returnVal = true;
    }
    prevPos = pos;
    prevT = millis();
    return returnVal;
}
int g_pidTurnLimit = 12000;
bool pidTurn(const double angle, const int wait) {
    turnPid.sensVal = odometry.getA();
    turnPid.target = angle;
    int pwr = clamp((int)turnPid.update(), -g_pidTurnLimit, g_pidTurnLimit);
    setDL(-pwr);
    setDR(pwr);
    if (turnPid.doneTime + wait < millis()) return true;
    return false;
}
bool pidTurnSweep(double tL, double tR, int wait) {
    DLPid.sensVal = getDL();
    DRPid.sensVal = getDR();
    DLPid.target = tL * ticksPerInch;
    DRPid.target = tR * ticksPerInch;
    setDL(DLPid.update());
    setDR(DRPid.update());
    if (DLPid.doneTime + wait < millis() && DRPid.doneTime + wait < millis()) return true;
    return false;
}

bool pidDriveArc(Point target, double rMag, int rotationDirection, int wait) {
    // error detection
    Point pos(odometry.getX(), odometry.getY());
    Point deltaPos = target - pos;
    Point midPt((pos.x + target.x) / 2.0, (pos.y + target.y) / 2.0);
    double altAngle = atan2(deltaPos.y, deltaPos.x) + (PI / 2) * rotationDirection;
    double altMag = sqrt(pow(rMag, 2) - pow(deltaPos.mag() / 2, 2));
    Point center = midPt + polarToRect(altMag, altAngle);
    double arcLen = 2 * acos(altMag / rMag) * rMag;
    drivePid.sensVal = arcLen;
    drivePid.target = 0;
    Point rVector = center - pos;
    Point targetVector = rVector.rotate(-1 * rotationDirection);
    Point orientationVector(cos(odometry.getA()), sin(odometry.getA()));
    double errAngle = acos((targetVector * orientationVector) / (targetVector.mag() * orientationVector.mag()));

    // allow for driving backwards
    int driveDir = 1;
    if (errAngle > PI / 2) {
        driveDir = -1;
        errAngle = PI - errAngle;
    }
    if (orientationVector < targetVector) errAngle *= -driveDir;
    if (orientationVector > targetVector) errAngle *= driveDir;

    // error correction
    double errRadius = (pos - center).mag() - rMag;
    turnPid.sensVal = errAngle;
    turnPid.target = 0;
    drivePid.sensVal *= cos(errAngle);
    double drivePwrFactor = 1.0 / (1.0 + errRadius);
    int drivePwr = clamp((int)(-drivePid.update() * drivePwrFactor), -8000, 8000);
    int turnPwr = clamp((int)turnPid.update(), -8000, 8000);
    if (fabs(arcLen) < 4) turnPwr = 0;
    setDL(drivePwr * driveDir - turnPwr);
    setDR(drivePwr * driveDir + turnPwr);

    if (drivePid.doneTime + wait < millis()) return true;
    return false;
}