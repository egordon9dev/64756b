#ifndef SETUP_H
#define SETUP_H
#include "main.h"
#include "pid.hpp"

#define TICKS_TO_DEG 0.4
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

int clamp(int n, int min, int max) { return n < min ? min : (n > max ? max : n); }

void setDR(int n) {
    n = clamp(n, -12000, 12000);
    mtr6.move_voltage(n);
    mtr7.move_voltage(n);
}
void setDL(int n) {
    n = clamp(n, -12000, 12000);
    mtr8.move_voltage(-n);
    mtr9.move_voltage(-n);
}
void intakeNone() { mtr5.move_voltage(0); }
void intakeFront() { mtr5.move_voltage(12000); }
void intakeAll() { mtr5.move_voltage(-12000); }

//----------- DRFB functions ---------
void setDrfb(int n) {
    clamp(n, -12000, 12000);
    n = drfbSlew.update(n);
    mtr12.move_voltage(-n);
}
double getDrfb() { return -mtr12.get_position(); }
void pidDrfb() {}  // 2470, 50
//---------- Claw functions --------
void setClaw(int n) {
    clamp(n, -12000, 12000);
    mtr11.move_voltage(n);
}
double getClaw() { return mtr11.get_position(); }
bool pidClawAbsolute(double a, long wait) {
    clawPid.target = a;
    clawPid.sensVal = getClaw();
    setClaw(clawPid.update());
    if (clawPid.doneTime + wait < millis()) return true;
    return false;
}  // 1350
bool pidClaw(double a, long wait, int id) {
    static int prevId = -1;
    static double target = 0;
    if (id != prevId) target = getClaw() + a;
    prevId = id;
    return pidClawAbsolute(target, wait);
}
//--------- Flywheel functions --------
void setFlywheel(int n) {
    n = clamp(n, -12000, 12000);
    n = flywheelSlew.update(n);
    mtr10.move_voltage(n);
}
double getFlywheel() { return mtr10.get_position(); }

void setup() {
    flywheelSlew.slewRate = 3;
    flywheelPid.kp = 1.0;
    clawPid.kp = 50.0;
}
#endif