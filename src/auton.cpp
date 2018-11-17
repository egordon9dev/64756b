#include "main.h"
#include "setup.hpp"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
using pros::delay;
using pros::millis;
using std::cout;
void auton1(bool leftSide) {
    int i = 0;
    Point targetPos(0, 45);
    double targetAngle = PI / 2;
    const int driveT = 200;
    IntakeState is = IntakeState::NONE;
    int t0 = BIL;
    while (1) {
        int j = 0;
        odometry.update();
        if (i == j++) {
            flywheelPid.target = 2.0;
            drfbPid.target = 1800;
            is = isBallIn() ? IntakeState::NONE : IntakeState::ALL;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                targetPos.y -= 10;
                i++;
            }
        } else if (i == j++) {
            is = isBallIn() ? IntakeState::NONE : IntakeState::ALL;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                targetAngle -= PI * 0.4;
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos0;
            is = isBallIn() ? IntakeState::NONE : IntakeState::ALL;
            if (pidTurn(targetAngle, driveT)) {
                turnPid.doneTime = BIL;
                targetPos = targetPos + polarToRect(-32, targetAngle);
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos0 + 50;
            is = IntakeState::NONE;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                i++;
            }
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        }
        pidClaw();
        pidFlywheel();
        pidDrfb();
        setIntake(is);
        delay(10);
    }
}
void autonomous() {
    setupAuton();
    auton1(true);
    stopMotors();
}
