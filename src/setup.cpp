#include "setup.hpp"
#include <string>
#include "MotorSaver.hpp"
#include "Point.hpp"
#include "main.h"
#include "pid.hpp"

// motors
pros::Motor mtr5(4);
pros::Motor mtr6(7);
pros::Motor mtr7(6);
pros::Motor mtr8(8);
pros::Motor mtr9(9);
pros::Motor mtr10(10);
pros::Motor mtr11(13);
pros::Motor mtr12(2);
// bad ports: 11, 12, 5, 1, 13

// motor savers
MotorSaver dlSaver(35);
MotorSaver drSaver(35);
MotorSaver drfbSaver(40);
MotorSaver clawSaver(65);
MotorSaver intakeSaver(40);
MotorSaver flySaver(40);

pros::Controller ctlr(pros::E_CONTROLLER_MASTER);
using pros::delay;
// sensors
pros::ADIPotentiometer* drfbPot;
pros::ADIPotentiometer* clawPot;
pros::ADILineSensor* ballSens;

//----------- Constants ----------------
const int drfbMaxPos = 3260, drfbPos0 = /*1390*/ 1105, drfbMinPos = 1070, drfbPos1 = 2280, drfbPos2 = 2798, drfbMinClaw = 1400, drfb18Max = 1445;
const int dblClickTime = 450, claw180 = 1350, clawPos0 = 590, clawPos1 = 3800;
const double ticksPerInch = 52.746 /*very good*/, ticksPerRadian = 368.309;
const double PI = 3.14159265358979323846;
const int BIL = 1000000000, MIL = 1000000;
int clamp(int n, int a, int b) { return n < a ? a : (n > b ? b : n); }
double clamp(double n, double a, double b) { return n < a ? a : (n > b ? b : n); }
Point polarToRect(double mag, double angle) {
    Point p(mag * cos(angle), mag * sin(angle));
    return p;
}
//----------- Drive -----------
double getDL() { return (-mtr8.get_position() - mtr9.get_position()) * 0.5; }
double getDR() { return (mtr6.get_position() + mtr7.get_position()) * 0.5; }
int millis() { return pros::millis(); }
void setDR(int n) {
    n = DRSlew.update(n);
    n = drSaver.getPwr(n, getDR());
    mtr6.move_voltage(n);
    mtr7.move_voltage(n);
}
void setDL(int n) {
    n = DLSlew.update(n);
    n = dlSaver.getPwr(n, getDL());
    mtr8.move_voltage(-n);
    mtr9.move_voltage(-n);
}
void printDriveEncoders() { printf("encs %d %d %d %d\n", (int)mtr6.get_position(), (int)mtr7.get_position(), (int)mtr8.get_position(), (int)mtr9.get_position()); }
int getDLVoltage() { return -mtr8.get_voltage(); }
int getDRVoltage() { return mtr6.get_voltage(); }
void runMotorTest() {
    for (int i = 0; i < 3; i++) {
        mtr6.move_voltage(5000);
        delay(150);
        stopMotors();
        delay(500);
        mtr7.move_voltage(5000);
        delay(150);
        stopMotors();
        delay(500);
        mtr8.move_voltage(5000);
        delay(150);
        stopMotors();
        delay(500);
        mtr9.move_voltage(5000);
        delay(150);
        stopMotors();
        delay(1500);
    }
}

//------------ Intake ---------------
void setIntake(int n) {
    n = clamp(n, -12000, 12000);
    static int prevFly = getFlywheel();
    if (getFlywheel() - prevFly < 15 && n < 0) n = 0;
    n = intakeSaver.getPwr(n, mtr5.get_position());
    mtr5.move_voltage(n);
    prevFly = getFlywheel();
}
void intakeNone() { setIntake(0); }
void intakeFront() { setIntake(12000); }
void intakeAll() { setIntake(-12000); }
void setIntake(IntakeState is) {
    if (is == IntakeState::NONE) {
        intakeNone();
    } else if (is == IntakeState::FRONT) {
        intakeFront();
    } else if (is == IntakeState::ALL) {
        intakeAll();
    }
}
int getBallSens() { return ballSens->get_value(); }
bool isBallIn() { return getBallSens() < 1800; }
//----------- DRFB functions ---------
void setDrfb(int n) {
    if (getDrfb() < drfbMinPos + 150 && n < -3500) n = -3500;
    if (getDrfb() > drfbMaxPos - 150 && n > 3500) n = 3500;
    n = drfbSlew.update(n);
    n = drfbSaver.getPwr(n, getDrfb());
    mtr12.move_voltage(-n);
}
int getDrfb() { return 4095 - drfbPot->get_value(); }
int getDrfbEncoder() { return -mtr12.get_position(); }
int getDrfbVoltage() { return mtr12.get_voltage(); }
bool pidDrfb(double pos, int wait) {
    drfbPid.target = pos;
    drfbPid.sensVal = getDrfb();
    drfbPid.sensVal = clamp((int)drfbPid.sensVal, drfbMinPos, drfbMaxPos);
    int out = drfbPid.update();
    setDrfb(out);
    if (drfbPid.doneTime + wait < millis()) return true;
}
void pidDrfb() { pidDrfb(drfbPid.target, 9999999); }
//---------- Claw functions --------
void setClaw(int n) {
    if (getDrfb() < drfbMinClaw || (getClaw() > clawPos1 && n > 0) || (getClaw() < clawPos0 && n < 0)) n = 0;
    n = clawSaver.getPwr(n, mtr11.get_position());
    n = clawSlew.update(n);
    mtr11.move_voltage(n);
}
double getClaw() { return clawPot->get_value(); }
int getClawVoltage() { return mtr11.get_voltage(); }
bool pidClaw(double a, int wait) {
    clawPid.target = a;
    clawPid.sensVal = getClaw();
    setClaw(clawPid.update());
    if (clawPid.doneTime + wait < millis()) return true;
    return false;
}  // 1350
void pidClaw() { pidClaw(clawPid.target, 999999); }
//--------- Flywheel functions --------
void setFlywheel(int n) {
    n = clamp(n, 0, 12000);
    n = flywheelSlew.update(n);
    n = flySaver.getPwr(n, getFlywheel());
    mtr10.move_voltage(n);
}
double getFlywheel() { return mtr10.get_position(); }
int getFlywheelVoltage() { return mtr10.get_voltage(); }
bool pidFlywheel(double speed) {
    flywheelPid.target = speed;
    static double prevFlywheelPos = 0;
    static int prevFlywheelT = 0;
    static int flywheelOutput = 0;
    int dt = millis() - prevFlywheelT;
    if (dt > 30) {
        if (dt < 500) {
            flywheelPid.sensVal = (getFlywheel() - prevFlywheelPos) / dt;
            flywheelOutput += flywheelPid.update();
            flywheelOutput = clamp(flywheelOutput, 0, 12000);
        }
        prevFlywheelPos = getFlywheel();
        prevFlywheelT = millis();
    }
    setFlywheel(flywheelOutput);
    return flywheelPid.doneTime < millis();
}
bool pidFlywheel(double speed, int wait) {
    pidFlywheel(speed);
    return flywheelPid.doneTime + wait < millis();
}
bool pidFlywheel() { return pidFlywheel(flywheelPid.target); }
//--------------------- Misc -----------------
const int ctlrIdxLeft = 0, ctlrIdxUp = 1, ctlrIdxRight = 2, ctlrIdxDown = 3, ctlrIdxY = 4, ctlrIdxX = 5, ctlrIdxA = 6, ctlrIdxB = 7, ctlrIdxL1 = 8, ctlrIdxL2 = 9, ctlrIdxR1 = 10, ctlrIdxR2 = 11;
const std::string clickIdxNames[] = {"Left", "Up", "Right", "Down", "Y", "X", "A", "B", "L1", "L2", "R1", "R2"};
const pros::controller_digital_e_t clickIdxIds[] = {DIGITAL_LEFT, DIGITAL_UP, DIGITAL_RIGHT, DIGITAL_DOWN, DIGITAL_Y, DIGITAL_X, DIGITAL_A, DIGITAL_B, DIGITAL_L1, DIGITAL_L2, DIGITAL_R1, DIGITAL_R2};
bool** getAllClicks() {
    static bool prevClicks[] = {false, false, false, false, false, false, false, false, false, false, false, false};
    static int prevTimes[] = {-9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999};
    bool curClicks[12];
    static bool dblClicks[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
    bool** allClicks = new bool*[3];
    for (int i = 0; i < 3; i++) { allClicks[i] = new bool[12]; }
    for (int i = 0; i < 12; i++) {
        curClicks[i] = ctlr.get_digital(clickIdxIds[i]);
        if (!curClicks[i]) dblClicks[i] = false;
        if (curClicks[i] && !prevClicks[i]) {
            // double tap
            if (millis() - prevTimes[i] < dblClickTime) dblClicks[i] = true;
            prevTimes[i] = millis();
        }
        allClicks[0][i] = prevClicks[i];
        allClicks[1][i] = curClicks[i];
        allClicks[2][i] = dblClicks[i];
        prevClicks[i] = curClicks[i];
    }
    return allClicks;
}
void printAllClicks(int line, bool** allClicks) {
    std::string line1 = "prevClicks: ";
    for (int i = 0; i < 12; i++) { line1 += (allClicks[0][i] ? (clickIdxNames[i] + ", ") : ""); }
    line1 = line1.substr(0, line1.size() - 2);
    std::string line2 = "curClicks: ";
    for (int i = 0; i < 12; i++) { line2 += (allClicks[1][i] ? (clickIdxNames[i] + ", ") : ""); }
    line2 = line2.substr(0, line2.size() - 2);
    std::string line3 = "dblClicks: ";
    for (int i = 0; i < 12; i++) { line3 += (allClicks[2][i] ? (clickIdxNames[i] + ", ") : ""); }
    line3 = line3.substr(0, line3.size() - 2);
    pros::lcd::print(line, line1.c_str());
    pros::lcd::print(line + 1, line2.c_str());
    pros::lcd::print(line + 2, line3.c_str());
}

void stopMotors() {
    setDrfb(0);
    setDL(0);
    setDR(0);
    setClaw(0);
    setFlywheel(0);
    intakeNone();
}
void printPidValues() {
    printf("%.1f drfb%2d %4d/%4d fly%2d %1.1f/%1.1f claw%2d %4d/%4d\n", millis() / 1000.0, (int)(getDrfbVoltage() / 1000 + 0.5), (int)drfbPid.sensVal, (int)drfbPid.target, getFlywheelVoltage() / 1000, flywheelPid.sensVal, flywheelPid.target, (int)(getClawVoltage() / 1000 + 0.5), (int)clawPid.sensVal, (int)clawPid.target);
    std::cout << std::endl;
}
extern Point g_target;
void printDrivePidValues() {
    printf("%.1f DL%d DR%d drive %3.2f/%3.2f turn %2.2f/%2.2f x %3.2f/%3.2f y %3.2f/%3.2f a %.2f\n", millis() / 1000.0, (int)(getDLVoltage() / 100 + 0.5), (int)(getDRVoltage() / 100 + 0.5), drivePid.sensVal, drivePid.target, turnPid.sensVal, turnPid.target, odometry.getX(), g_target.x, odometry.getY(), g_target.y, odometry.getA());
    std::cout << std::endl;
}

void setDrfbParams(bool auton) {
    if (auton) {
        drfbPid.kp = 20.0;
        drfbPid.ki = 0.1;
        drfbPid.iActiveZone = 150;
        drfbPid.maxIntegral = 3000;
        drfbPid.kd = 75;
    } else {
        drfbPid.kp = 7.0;
        drfbPid.ki = 0.0;
        drfbPid.kd = 0.0;
    }
}
void setupAuton() {
    flywheelSlew.slewRate = 9999;  // 60;
    flywheelPid.kp = 200.0;
    flywheelPid.kd = 10000.0;
    flywheelPid.dInactiveZone = 0.01;
    flywheelPid.DONE_ZONE = 0.2;
    flySaver.setConstants(1, 1, 0, 0);

    clawPid.kp = 10.0;
    clawPid.ki = 0.0;
    clawPid.iActiveZone = 300;
    clawPid.unwind = 0;
    clawSlew.slewRate = 200;

    drfbSlew.slewRate = 99999;
    setDrfbParams(true);
    drfbSaver.setConstants(0.0, 0.5, 0.0, 0.01);
    drfbPid.DONE_ZONE = 100;
    drfbPid.target = drfbPos0;

    DLSlew.slewRate = 120;
    DRSlew.slewRate = 120;
    DLPid.kp = DRPid.kp = 900;
    DLPid.kd = DRPid.kd = 25000;
    DLPid.DONE_ZONE = DRPid.DONE_ZONE = 1.5;

    drivePid.kp = 900;
    drivePid.kd = 25000;
    drivePid.DONE_ZONE = 3.0;
    turnPid.kp = 15000;
    turnPid.DONE_ZONE = PI / 20;

    curvePid.kp = 100000;
    curvePid.ki = 2000;
    curvePid.kd = 400000;  // was 3000000
    curvePid.iActiveZone = PI / 18;
    curvePid.maxIntegral = 5000;

    drfbPid.target = drfbPos0;
    clawPid.target = clawPos1;
    flywheelPid.target = 0;
    static bool first = true;
    if (first) {
        drfbPot = new pros::ADIPotentiometer(2);
        clawPot = new pros::ADIPotentiometer(4);
        ballSens = new pros::ADILineSensor(8);
        printf("\n\n\ndynamics inited.\n\n\n");
    }
    first = false;
}
void setupOpCtrl() { setDrfbParams(false); }