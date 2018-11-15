#include "setup.hpp"
#include <string>
#include "main.h"
#include "pid.hpp"

using namespace pros;
pros::Motor mtr5(5);
pros::Motor mtr6(6);
pros::Motor mtr7(7);
pros::Motor mtr8(8);
pros::Motor mtr9(9);
pros::Motor mtr10(10);
pros::Motor mtr11(11);
pros::Motor mtr12(12);
pros::Controller ctlr(pros::E_CONTROLLER_MASTER);

pros::ADIPotentiometer* drfbPot;
pros::ADILineSensor* ballSens;

//----------- Constants ----------------
const int drfbMaxPos = 3882, drfbPos0 = /*1390*/ 1370, drfbMinPos = 1350, drfbPos1 = 2675, drfbPos2 = 3125, drfbMinClaw = 1600;
const int dblClickTime = 450;
const double ticksPerInch = 52.746 /*very good*/, ticksPerRadian = 368.309;
const double PI = 3.14159265358979323846;
int clamp(int n, int min, int max) { return n < min ? min : (n > max ? max : n); }

//----------- Drive -----------
void setDR(int n) {
    n = clamp(n, -12000, 12000);
    n = DRSlew.update(n);
    mtr6.move_voltage(n);
    mtr7.move_voltage(n);
}
void setDL(int n) {
    n = clamp(n, -12000, 12000);
    n = DLSlew.update(n);
    mtr8.move_voltage(-n);
    mtr9.move_voltage(-n);
}
double getDL() { return (-mtr8.get_position() - mtr9.get_position()) * 0.5; }
double getDR() { return (mtr6.get_position() + mtr7.get_position()) * 0.5; }

//------------ Intake ---------------
void intakeNone() { mtr5.move_voltage(0); }
void intakeFront() { mtr5.move_voltage(12000); }
void intakeAll() { mtr5.move_voltage(-12000); }
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
//----------- DRFB functions ---------

void setDrfb(int n) {
    static int prevN = -99999, t0 = 0;
    static int prevDrfb = getDrfb(), prevT = millis();
    drfbPid.update();
    if (abs(n - prevN) > 1000) {
        t0 = millis();
        prevN = n;
    }
    double vel = (double)(getDrfb() - prevDrfb) / (millis() - prevT);
    prevT = millis();
    prevDrfb = getDrfb();
    bool stall = millis() - t0 > 50 && ((n < 0 && vel > -0.1) || (n > 0 && vel < 0.1));
    int maxStall = stall ? 3000 : 12000;
    if ((getDrfb() < drfbMinPos + 300 || getDrfb() > drfbMaxPos - 300) && stall > 4500) stall = 4500;
    // n += (getDrfb() - drfbMinPos) / 3.0 - 300;
    n = clamp(n, -maxStall, maxStall);
    n = drfbSlew.update(n);
    mtr12.move_voltage(-n);
}
int getDrfb() { return 4095 - drfbPot->get_value(); }
int getDrfbVoltage() { return mtr12.get_voltage(); }
bool pidDrfb(double pos, int wait) {
    drfbPid.target = pos;
    drfbPid.sensVal = getDrfb();
    drfbPid.sensVal = clamp(drfbPid.sensVal, drfbMinPos, drfbMaxPos);
    int out = drfbPid.update();
    setDrfb(out);
    if (drfbPid.doneTime + wait < millis()) return true;
}
void pidDrfb() { pidDrfb(drfbPid.target, 9999999); }
//---------- Claw functions --------
void setClaw(int n) {
    clamp(n, -12000, 12000);
    mtr11.move_voltage(n);
}
double getClaw() { return mtr11.get_position(); }
bool pidClaw(double a, int wait) {
    clawPid.target = a;
    clawPid.sensVal = getClaw();
    setClaw(clawPid.update());
    if (clawPid.doneTime + wait < millis()) return true;
    return false;
}  // 1350
//--------- Flywheel functions --------
void setFlywheel(int n) {
    n = clamp(n, 0, 12000);
    n = flywheelSlew.update(n);
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
        }
        prevFlywheelPos = getFlywheel();
        prevFlywheelT = millis();
    }
    setFlywheel(flywheelOutput);
    return fabs(flywheelPid.sensVal - flywheelPid.target) < flywheelPid.DONE_ZONE;
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

void printPidValues() {
    printf("%.1f drfb%2d %4f/%4f fly%2d %1.1f/%1.1f\n", millis() / 1000.0, (int)(getDrfbVoltage() / 1000 + 0.5), drfbPid.sensVal, drfbPid.target, getFlywheelVoltage() / 1000, flywheelPid.sensVal, flywheelPid.target);
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
    flywheelPid.kp = 130.0;
    flywheelPid.kd = 100000.0;
    flywheelPid.dInactiveZone = 0.01;

    flywheelPid.DONE_ZONE = 0.2;

    clawPid.kp = 50.0;

    drfbSlew.slewRate = 99999;
    setDrfbParams(true);

    DLSlew.slewRate = 120;
    DRSlew.slewRate = 120;

    drfbPot = new ADIPotentiometer(2);
    ballSens = new ADILineSensor(8);
}

void setupOpCtrl() { setDrfbParams(false); }