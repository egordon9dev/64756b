#include "main.h"
#include "pid.hpp"
#include "setup.hpp"
using namespace pros;
/*
Ch3         drive
Ch1         turn
R1          lift auto up
R2          lift auto down
L1          intake 1 ball
UP          shoot 1 ball, intake next ball
A, Y        flip  cap
*/

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    setup();
    double drv[] = {0, 0};
    int prevT = 0;
    int dt = 0;
    double prevFlywheel = 0, dFlywheel = 0;
    bool prevDY = false, prevDA = false, prevR1 = false, prevR2 = false, prevL1 = false, prevL2 = false;
    int prevL2T = 0;
    int clawCtr = 0;
    bool drfbPidRunning = false;
    IntakeState intakeState = IntakeState::NONE;
    int ballSensT = INT_MAX;
    while (true) {
        dt = pros::millis() - prevT;
        prevT = pros::millis();
        pros::lcd::print(0, "%.2lfv      %d%%", pros::battery::get_voltage() / 1000.0, (int)pros::battery::get_capacity());
        pros::lcd::print(1, "drfb %d", getDrfb());
        pros::lcd::print(2, "ballSens %d", getBallSens());
        pros::lcd::print(3, "claw %d", getClaw());
        pros::lcd::print(4, "flywheel %d", getFlywheel());
        // std::cout << getDrfb() << std::endl;
        // DRIVE
        int joy[] = {(int)(ctlr.get_analog(ANALOG_RIGHT_X) * 12000.0 / 127.0), (int)(ctlr.get_analog(ANALOG_LEFT_Y) * 12000.0 / 127.0)};
        // std::cout << joy[0] << ", " << joy[1] << std::endl;
        dFlywheel = getFlywheel() - prevFlywheel;
        prevFlywheel = getFlywheel();
        if (abs(joy[0]) < 10) joy[0] = 0;
        if (abs(joy[1]) < 10) joy[1] = 0;
        double drv[] = {0.1 * joy[1] + 0.9 * drv[0], joy[0] * 0.1 + 0.9 * drv[1]};
        setDL(drv[0] + drv[1]);
        setDR(drv[0] - drv[1]);

        // FLYWHEEL
        if (ctlr.get_digital(DIGITAL_UP)) {
            pidFlywheel(2.9);
        } else if (ctlr.get_digital(DIGITAL_RIGHT)) {
            pidFlywheel(2.5);
        } else if (ctlr.get_digital(DIGITAL_DOWN)) {
            pidFlywheel(0);
        } else {
            pidFlywheel();
        }

        // drfb
        bool curR1 = ctlr.get_digital(DIGITAL_R1), curR2 = ctlr.get_digital(DIGITAL_R2);
        double drfbPos = getDrfb();
        // drfb (low to high)=(1395 to 3882)
        if (curR1) {
            drfbPidRunning = true;
            pidDrfb(3200, 999999);
            // 2470
        } else if (curR2) {
            drfbPidRunning = true;
            pidDrfb(1420, 999999);
        } else {
            if (drfbPidRunning) {
                pidDrfb();
            } else {
                setDrfb(0);
            }
        }

        // CLAW
        bool curDY = ctlr.get_digital(DIGITAL_Y);
        bool curDA = ctlr.get_digital(DIGITAL_A);
        if ((!prevDY && curDY) || (!prevDA && curDA)) clawCtr++;
        prevDY = curDY;
        prevDA = curDA;
        int claw180 = 1350;
        double curClaw = getClaw();
        if (curDY && curClaw > drfbMinClaw) {
            pidClaw((int)((curClaw + claw180 * 1.5 + ((curClaw < 0) ? -claw180 : 0)) / claw180) * claw180 - curClaw, 999999, clawCtr);
        } else if (curDA && curClaw > drfbMinClaw) {
            pidClaw((int)((curClaw - claw180 * 0.5 + ((curClaw < 0) ? -claw180 : 0)) / claw180) * claw180 - curClaw, 999999, clawCtr);
        } else if (ctlr.get_digital(DIGITAL_X)) {
            pidClaw(10, 999999, -777);
        } else {
            pidClaw(0, 999999, clawCtr);
        }
        bool curL1 = ctlr.get_digital(DIGITAL_L1), curL2 = ctlr.get_digital(DIGITAL_L2);
        if (curL2 && !prevL2) {
            // double tap
            if (millis() - prevT < 600) { intakeState = IntakeState::NONE; }
            prevL2T = millis();
        }
        // INTAKE
        if (curL1) {
            intakeState = IntakeState::ALL;
        } else if (curL2) {
            intakeState = IntakeState::FRONT;
        }
        if (!curL1 || !curL2) {
            if (getBallSens() < 1900 && intakeState == IntakeState::ALL) {
              if(ballSensT > millis()) ballSensT = millis();
              if(millis() - ballSensT > 200) {
                intakeState = IntakeState::FRONT;
              }
            }
        }
        setIntake(intakeState);
        prevL1 = curL1;
        prevL2 = curL2;
        if (ctlr.get_digital(DIGITAL_B)) return;
        pros::delay(10);
    }
    delete drfbPot;
    delete ballSens;
}
