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
    bool prevDY = false, prevDA = false, prevR1 = false, prevR2 = false, prevL1 = false, prevL2 = false, prevX = false, prevB = false;
    int prevL2T = -9999999;
    int clawCtr = 0;
    bool drfbPidRunning = false;
    IntakeState intakeState = IntakeState::NONE;
    int driveDir = 1;
    std::cout << "-1" << std::endl;
    while (true) {
        dt = pros::millis() - prevT;
        prevT = pros::millis();
        pros::lcd::print(0, "%.2lfv      %d%%", pros::battery::get_voltage() / 1000.0, (int)pros::battery::get_capacity());
        pros::lcd::print(1, "drfb %d", getDrfb());
        pros::lcd::print(2, "ballSens %d", getBallSens());
        pros::lcd::print(3, "claw %d", getClaw());
        pros::lcd::print(4, "flywheel %d", getFlywheel());
        bool** allClicks = getAllClicks();
        bool prevClicks[12], curClicks[12], dblClicks[12];
        for (int i = 0; i < 12; i++) {
            prevClicks[i] = allClicks[0][i];
            curClicks[i] = allClicks[1][i];
            dblClicks[i] = allClicks[2][i];
        }
        printAllClicks(5, allClicks);

        if (curClicks[ctlrIdxB] && !prevClicks[ctlrIdxB]) { driveDir *= -1; }
        // std::cout << getDrfb() << std::endl;
        // DRIVE
        int joy[] = {(int)(ctlr.get_analog(ANALOG_RIGHT_X) * 12000.0 / 127.0), (int)(driveDir * ctlr.get_analog(ANALOG_LEFT_Y) * 12000.0 / 127.0)};
        // std::cout << joy[0] << ", " << joy[1] << std::endl;
        dFlywheel = getFlywheel() - prevFlywheel;
        prevFlywheel = getFlywheel();
        if (abs(joy[0]) < 10) joy[0] = 0;
        if (abs(joy[1]) < 10) joy[1] = 0;
        double drv[] = {0.1 * joy[1] + 0.9 * drv[0], joy[0] * 0.1 + 0.9 * drv[1]};
        setDL(drv[0] + drv[1]);
        setDR(drv[0] - drv[1]);

        // FLYWHEEL
        if (curClicks[ctlrIdxUp]) {
            pidFlywheel(2.9);
        } else if (curClicks[ctlrIdxRight]) {
            pidFlywheel(2.5);
        } else if (curClicks[ctlrIdxDown]) {
            pidFlywheel(0);
        } else {
            pidFlywheel();
        }

        // drfb
        double drfbPos = getDrfb();
        if (dblClicks[ctlrIdxR1]) {
            drfbPidRunning = true;
            pidDrfb(drfbPos2, 999999);
        } else if (curClicks[ctlrIdxR1]) {
            drfbPidRunning = true;
            pidDrfb(drfbPos1, 999999);
            // 2470
        } else if (curClicks[ctlrIdxR2]) {
            drfbPidRunning = true;
            pidDrfb(1390, 999999);
        } else {
            if (drfbPidRunning) {
                pidDrfb();
            } else {
                setDrfb(0);
            }
        }

        // CLAW
        if (!prevClicks[ctlrIdxX] && curClicks[ctlrIdxX]) clawCtr++;
        int claw180 = 1350;
        double curClaw = getClaw();
        if (curClicks[ctlrIdxX] && curClaw > drfbMinClaw) {
            pidClaw((int)((curClaw + claw180 * 1.5 + ((curClaw < 0) ? -claw180 : 0)) / claw180) * claw180 - curClaw, 999999, clawCtr);
        }
        /*else if (curDA && curClaw > drfbMinClaw) {
            pidClaw((int)((curClaw - claw180 * 0.5 + ((curClaw < 0) ? -claw180 : 0)) / claw180) * claw180 - curClaw, 999999, clawCtr);
        } */
        else {
            pidClaw(0, 999999, clawCtr);
        }

        if (dblClicks[ctlrIdxL2]) intakeState = IntakeState::NONE;
        // INTAKE
        if (curClicks[ctlrIdxL1]) {
            intakeState = IntakeState::ALL;
        } else if (curClicks[ctlrIdxL2]) {
            intakeState = IntakeState::FRONT;
        }
        if (!curClicks[ctlrIdxL1] || !curClicks[ctlrIdxL2]) {
            if (getBallSens() < 1800 && intakeState == IntakeState::ALL) intakeState = IntakeState::FRONT;
        } else {
            intakeState = IntakeState::ALL;
        }
        setIntake(intakeState);

        delete[] allClicks[0];
        delete[] allClicks[1];
        delete[] allClicks[2];
        delete[] allClicks;
        pros::delay(10);
    }
    delete drfbPot;
    delete ballSens;
}