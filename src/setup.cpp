#include "setup.hpp"
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
const int drfbMinPos = 1395, drfbMaxPos = 3882, drfbMinClaw = 1600;

int clamp(int n, int min, int max) { return n < min ? min : (n > max ? max : n); }

//----------- Drive -----------
void setDR(int n) {
    n = clamp(n, -12000, 12000);
    mtr6.move_voltage(n);
    mtr7.move_voltage(n);
}
int getDR() { return mtr7.get_position(); }
void setDL(int n) {
    n = clamp(n, -12000, 12000);
    mtr8.move_voltage(-n);
    mtr9.move_voltage(-n);
}
int getDL() { return -mtr9.get_position(); }
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
    clamp(n, -12000, 12000);
    n = drfbSlew.update(n);
    mtr12.move_voltage(-n);
}
double getDrfb() { return 4095 - drfbPot->get_value(); }
bool pidDrfb(double pos, int wait) {
    drfbPid.target = pos;
    drfbPid.sensVal = getDrfb();
    drfbPid.sensVal = clamp(drfbPid.sensVal, drfbMinPos, drfbMaxPos);
    int out = drfbPid.update();
    std::cout << "drfb " << out << " " << drfbPid.sensVal << "/" << drfbPid.target << std::endl;
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
bool pidClawAbsolute(double a, int wait) {
    clawPid.target = a;
    clawPid.sensVal = getClaw();
    setClaw(clawPid.update());
    if (clawPid.doneTime + wait < millis()) return true;
    return false;
}  // 1350
bool pidClaw(double a, int wait, int id) {
    static int prevId = -1;
    static double target = 0;
    if (id != prevId) target = getClaw() + a;
    prevId = id;
    return pidClawAbsolute(target, wait);
}
//--------- Flywheel functions --------
void setFlywheel(int n) {
    n = clamp(n, 0, 12000);
    n = flywheelSlew.update(n);
    mtr10.move_voltage(n);
}
double getFlywheel() { return mtr10.get_position(); }
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
void setup() {
    flywheelSlew.slewRate = 3;
    flywheelPid.kp = 10.0;
    flywheelPid.DONE_ZONE = 0.2;

    clawPid.kp = 50.0;

    drfbPid.kp = 20.0;
    drfbPid.ki = 0.1;
    drfbPid.kd = 2;

    drfbPot = new ADIPotentiometer(2);
    ballSens = new ADILineSensor(8);
}

void updateOdometry() {

}
