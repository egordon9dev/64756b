#include "auton.hpp"
#include "main.h"
#include "pid.hpp"
#include "point.hpp"
#include "setup.hpp"
#include "test.hpp"
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
using pros::delay;
using std::cout;
using std::endl;
void opcontrol() {
    setupAuton();
    setupOpCtrl();
    if (pros::battery::get_capacity() < 10.0) {
        for (int i = 0; i < 8; i++) {
            pros::lcd::print(1, "LOW BATTERY");
            std::cout << "LOW BATTERY" << std::endl;
        }
        // return;
    }
    const int opcontrolT0 = millis();
    double drv[] = {0, 0};
    int prevT = 0;
    int dt = 0;
    double prevFlywheel = 0, dFlywheel = 0;
    bool prevDY = false, prevDA = false, prevR1 = false, prevR2 = false, prevL1 = false, prevL2 = false, prevX = false, prevB = false;
    int prevL2T = -9999999;
    int tDrfbOff = 0;
    bool drfbPidRunning = false;
    bool clawFlipped = false, clawInit = false;
    IntakeState intakeState = IntakeState::NONE;
    int driveDir = 1;
    int nBalls = 0;
    int intakeT0 = BIL;
    if (1) {
        auton2(true);
        while (1) delay(5000);
        // odometry.setA(0);
        // odometry.setX(-5);
        // odometry.setY(0);
        // pidDriveArcInit(Point(-5, 0), Point(-20, -12), 5, 1, 100);
        // while (!ctlr.get_digital(DIGITAL_B)) {
        //     odometry.update();
        //     printArcData();
        //     if (pidDriveArc()) break;
        //     delay(10);
        // }

        flywheelPid.target = 0;
        clawPid.target = getClaw();
        drfbPid.target = getDrfb();
        int i = 0, t0, sideSign = 1, driveT = 100;
        bool drfbPidRunning = true, clawPidRunning = true;
        IntakeState is = IntakeState::NONE;
        int lastT, autonT0 = millis();
        bool printing = false;
        Point ptA = Point(-5.3, 0), ptB;
        while (!ctlr.get_digital(DIGITAL_B)) {
            odometry.update();
            int j = 0;
            if (i == j++) {
                setDL(12000);
                setDR(4000);
                if (millis() - autonT0 > 1000) i++;
            } else if (i == j++) {
                odometry.setX(0);
                odometry.setY(0);
                odometry.setA(0);
                dlSaver.reset();
                drSaver.reset();
                pidDriveInit(ptA, 100);
                i++;
            } else if (i == j++) {
                printf("align for cap place ");
                if (pidDrive()) {
                    drfbPid.target = drfbPos0;
                    t0 = millis();
                    i++;
                }
            } else if (i == j++) {
                printf("place cap ");
                drfbPidRunning = false;
                setDL(0);
                setDR(0);
                setDrfb(-12000);
                printing = true;
                if (getDrfb() < drfbPos1 + 50 || millis() - t0 > 800) {
                    drfbPid.target = drfbPos1;
                    drfbPidRunning = true;
                    t0 = millis();
                    ptB = Point(-sideSign * 35, -8);
                    pidDriveArcInit(ptA, ptB, 55, sideSign, driveT);
                    i++;
                }
            } else if (i == j++) {
                printing = false;
                printf("arc to shoot pos ");
                printArcData();
                if (millis() - t0 > 300) drfbPid.target = drfbPos0;
                if (pidDriveArc()) { i++; }
            } else {
                stopMotors();
            }
            if (clawPidRunning) pidClaw();
            pidFlywheel();
            if (drfbPidRunning) pidDrfb();
            setIntake(is);
            if (millis() - lastT > 100 && printing) {
                printf("t%d ", millis() - autonT0);
                printDrivePidValues();
                lastT = millis();
            }
            delay(10);
            delay(10);
        }
        while (0) {
            for (int i = 0; i < 5; i++) {
                odometry.update();
                // pros::lcd::print(0, "x %f", odometry.getX());
                // pros::lcd::print(1, "y %f", odometry.getY());
                // pros::lcd::print(2, "a %f", odometry.getA());
                // printDrivePidValues();
                // printf("claw %d     drfb %d\n", (int)getClaw(), (int)getDrfb());
                pidDriveArc();
                delay(10);
            }
            printArcData();
        }
    }
    setDrfbParams(false);
    while (true) {
        dt = millis() - prevT;
        prevT = millis();
        pros::lcd::print(7, "%.2lfv      %d%%", pros::battery::get_voltage() / 1000.0, (int)pros::battery::get_capacity());
        // pros::lcd::print(1, "drfb %d", getDrfb());
        // pros::lcd::print(2, "ballSens %d", getBallSens());
        // pros::lcd::print(3, "claw %d", getClaw());
        // pros::lcd::print(4, "flywheel %d", getFlywheel());
        odometry.update();

        pros::lcd::print(0, "x %f", odometry.getX());
        pros::lcd::print(1, "y %f", odometry.getY());
        pros::lcd::print(2, "a %f", odometry.getA()); /*
         pros::lcd::print(3, "drfb %d", getDrfb());*/
        printPidValues();
        bool** allClicks = getAllClicks();
        bool prevClicks[12], curClicks[12], dblClicks[12];
        for (int i = 0; i < 12; i++) {
            prevClicks[i] = allClicks[0][i];
            curClicks[i] = allClicks[1][i];
            dblClicks[i] = allClicks[2][i];
        }
        // printAllClicks(5, allClicks);

        if (curClicks[ctlrIdxB] && !prevClicks[ctlrIdxB]) { driveDir *= -1; }
        // DRIVE
        int joy[] = {(int)(ctlr.get_analog(ANALOG_RIGHT_X) * 12000.0 / 127.0), (int)(driveDir * ctlr.get_analog(ANALOG_LEFT_Y) * 12000.0 / 127.0)};
        dFlywheel = getFlywheel() - prevFlywheel;
        prevFlywheel = getFlywheel();
        if (abs(joy[0]) < 10) joy[0] = 0;
        if (abs(joy[1]) < 10) joy[1] = 0;
        setDL(joy[1] + joy[0]);
        setDR(joy[1] - joy[0]);

        // FLYWHEEL
        if (curClicks[ctlrIdxDown]) {
            pidFlywheel(0);
        } else if (curClicks[ctlrIdxUp]) {
            pidFlywheel(2.9);
        } else {
            pidFlywheel();
        }

        // drfb
        double drfbPos = getDrfb();
        if (curClicks[ctlrIdxR1]) {
            drfbPidRunning = false;
            tDrfbOff = millis();
            setDrfb(12000);
        } else if (curClicks[ctlrIdxR2]) {
            drfbPidRunning = false;
            tDrfbOff = millis();
            setDrfb(-8000);
        } else if (curClicks[ctlrIdxY]) {
            drfbPidRunning = true;
            drfbPid.target = drfbPos1;
            setDrfbParams(true);
        } else if (curClicks[ctlrIdxA]) {
            drfbPidRunning = true;
            drfbPid.target = drfbPos2;
            setDrfbParams(true);
        } else if (millis() - tDrfbOff > 130 && millis() - opcontrolT0 > 300) {
            if (!drfbPidRunning) {
                drfbPidRunning = true;
                drfbPid.target = getDrfb();
                setDrfbParams(false);
            }
        } else if (!drfbPidRunning) {
            setDrfb(0);
        }
        if (drfbPidRunning) pidDrfb();

        // CLAW
        if (millis() - opcontrolT0 > 300) {
            if (clawInit) {
                if (curClicks[ctlrIdxX] && !prevClicks[ctlrIdxX]) {
                    clawFlipped = !clawFlipped;
                    if (getDrfb() < drfbMinClaw && drfbPid.target < drfbMinClaw) {
                        drfbPidRunning = true;
                        drfbPid.target = drfbMinClaw;
                        setDrfbParams(true);
                    }
                }
                clawPid.target = clawFlipped ? clawPos1 : clawPos0;
                pidClaw(clawPid.target, 999999);
            } else {
                clawFlipped = getClaw() > (clawPos0 + clawPos1) / 2;
                clawInit = true;
            }
        } else {
            setClaw(0);
        }
        // INTAKE
        if (curClicks[ctlrIdxL1] && curClicks[ctlrIdxL2]) {
            intakeState = IntakeState::ALL;
        } else {
            if (dblClicks[ctlrIdxL2]) {
                intakeState = IntakeState::NONE;
            } else if (curClicks[ctlrIdxL2] && !prevClicks[ctlrIdxL2]) {
                intakeState = IntakeState::ALL;
                intakeT0 = millis();
            }
            if (isBallIn() && intakeState == IntakeState::ALL && millis() - intakeT0 > 500) {
                intakeState = IntakeState::FRONT;
                nBalls++;
            }
        }
        if (flywheelPid.sensVal < 0.5 && intakeState == IntakeState::ALL) { intakeState = IntakeState::FRONT; }
        setIntake(intakeState);

        delete[] allClicks[0];
        delete[] allClicks[1];
        delete[] allClicks[2];
        delete[] allClicks;
        pros::delay(10);
    }
    delete drfbPot;
    delete clawPot;
    delete ballSens;
}