#include "main.h"
#include "pid.hpp"
#include "setup.hpp"
using namespace pros::literals;
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
    long prevT = 0;
    int dt = 0;
    double prevFlywheel = 0, dFlywheel = 0;
    bool prevDY = false, prevDA = false;
    int clawCtr = 0;
    std::cout << "the size is " << sizeof(int) << std::endl;
    while (true) {
        dt = pros::millis() - prevT;
        prevT = pros::millis();
        pros::lcd::print(0, "%.2lfv      %d%%", pros::battery::get_voltage() / 1000.0, (int)pros::battery::get_capacity());
        // pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2, (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1, (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        // std::cout << dFlywheel / dt << std::endl; //Flywheel
        std::cout << getClaw() << std::endl;  // Claw

        // DRIVE
        int joy[] = {(int)(ctlr.get_analog(ANALOG_RIGHT_X) * 12000.0 / 127.0), (int)(ctlr.get_analog(ANALOG_RIGHT_Y) * 12000.0 / 127.0)};
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
            setFlywheel(12000);
        } else {
            setFlywheel(0);
        }

        // drfb
        if (ctlr.get_digital(DIGITAL_R1)) {
            setDrfb(12000);
        } else if (ctlr.get_digital(DIGITAL_R2)) {
            setDrfb(-12000);
        } else {
            setDrfb(0);
        }

        // CLAW
        bool curDY = ctlr.get_digital(DIGITAL_Y);
        bool curDA = ctlr.get_digital(DIGITAL_A);
        if ((!prevDY && curDY) || (!prevDA && curDA)) clawCtr++;
        prevDY = curDY;
        prevDA = curDA;
        int claw180 = 1350;
        double curClaw = getClaw();
        if (curDY) {
            pidClaw((int)((curClaw + claw180 * 1.5 + ((curClaw < 0) ? -claw180 : 0)) / claw180) * claw180 - curClaw, 999999, clawCtr);
        } else if (curDA) {
            pidClaw((int)((curClaw - claw180 * 0.5 + ((curClaw < 0) ? -claw180 : 0)) / claw180) * claw180 - curClaw, 999999, clawCtr);
        } else if (ctlr.get_digital(DIGITAL_X)) {
            pidClaw(10, 999999, -777);
        } else {
            pidClaw(0, 999999, clawCtr);
        }

        // INTAKE
        if (ctlr.get_digital(DIGITAL_L1)) {
            intakeAll();
        } else if (ctlr.get_digital(DIGITAL_L2)) {
            intakeFront();
        } else {
            intakeNone();
        }

        pros::delay(10);
    }
}