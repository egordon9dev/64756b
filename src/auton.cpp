#include "Point.hpp"
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
using std::endl;
void auton1(bool leftSide) {
    int sideSign = leftSide ? 1 : -1;
    odometry.setXAxisDir(sideSign);
    odometry.setRotationDir(sideSign);
    int i = 0;
    odometry.setA(-PI / 2);
    Point targetPos(0, 42);
    double targetAngle = -PI / 2;
    const int driveT = 50;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = true;
    while (1) {
        int j = 0;
        odometry.update();
        if (i == j++) {
            flywheelPid.target = 2.0;
            drfbPid.target = 1800;
            is = IntakeState::FRONT;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.y -= 5;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetAngle -= PI * 0.37;
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos0;
            if (pidTurn(targetAngle, driveT)) {
                turnPid.doneTime = BIL;
                targetPos = targetPos - polarToRect(20, targetAngle).abs();
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos0 + 50;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x = -18;
                targetPos.y = 6;
                arcRadius = (targetPos - odometry.getPos()).mag() + 5;
                i++;
            }
        } else if (i == j++) {
            double distanceToTarget = (targetPos - odometry.getPos()).mag();
            if (arcRadius < distanceToTarget + 1) arcRadius = distanceToTarget + 1;
            if (pidDriveArc(targetPos, arcRadius, 1, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetAngle = odometry.getA() + PI * 0.22;
                i++;
            }
        } else if (i == j++) {
            if (pidTurn(targetAngle, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // SHOOT BALL 1
            setDL(0);
            setDR(0);
            is = IntakeState::ALL;
            if (millis() - t0 > 1000) {
                is = IntakeState::NONE;
                targetPos.x -= 22;
                flywheelPid.doneTime = BIL;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x += 37;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // SHOOT BALL 2
            setDL(0);
            setDR(0);
            is = IntakeState::ALL;
            if (millis() - t0 > 2000) {
                is = IntakeState::NONE;
                targetPos.x += 11;
                flywheelPid.target = 0;
                arcRadius = (targetPos - odometry.getPos()).mag() + 5;
                i++;
            }
        } else if (i == j++) {
            flywheelPid.target = 0;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                drfbPid.doneTime = BIL;
                targetAngle -= PI / 4;
                drfbPidRunning = false;
                i++;
            }
        } else if (i == j++) {
            clawPid.target = claw180;
            bool turnDone = pidTurn(targetAngle, 0);
            bool drfbDone = getDrfb() > drfbPos2 + 200;
            if (drfbDone) {
                drfbPid.target = getDrfb();
                drfbPidRunning = true;
            } else {
                setDrfb(12000);
            }
            if (turnDone && drfbDone) {
                drfbPid.doneTime = BIL;
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            if (millis() - t0 > 2000) {
                drfbPid.target = drfbPos2;
                setDL(0);
                setDR(0);
            } else {
                setDL(12000);
                setDR(1000);
            }
            if (drfbPid.doneTime < millis() && millis() - t0 > 2000) {
                drfbPid.doneTime = BIL;
                odometry.setX(0);
                odometry.setX(0);
                odometry.setA(-PI / 2);
                targetPos.x = 0;
                targetPos.y = 15;
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos2;
            if (pidDrive(targetPos, driveT)) {
                setDL(0);
                setDR(0);
                i++;
            }
        } else {
            stopMotors();
        }
        pidClaw();
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        setIntake(is);
        if (i != prevI) {
            for (int w = 0; w < 15; w++) cout << endl;
        }
        prevI = i;
        if (millis() - lastT > 100) {
            printDrivePidValues();
            lastT = millis();
        }
        delay(10);
    }
}
void auton2(bool leftSide) {
    int sideSign = leftSide ? 1 : -1;
    odometry.setXAxisDir(sideSign);
    odometry.setRotationDir(sideSign);
    int i = 0;
    odometry.setA(-PI / 2);
    Point targetPos(0, 42);
    double targetAngle = -PI / 2;
    const int driveT = 50;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = true;
    while (1) {
        int j = 0;
        odometry.update();
        if (i == j++) {
            drfbPid.target = drfbPos0;
            is = IntakeState::FRONT;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.y -= 5;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetAngle += PI * 0.62;
                i++;
            }
        } else if (i == j++) {
            if (pidTurn(targetAngle, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos = targetPos + polarToRect(24, targetAngle).abs();
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                drfbPidRunning = false;
                g_pidTurnLimit = 4000;
                targetAngle -= PI * 0.9;
                i++;
            }
        } else if (i == j++) {
            setDrfb(3000);
            if (pidTurn(targetAngle, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x = 17;
                targetPos.y = 12;
                g_pidTurnLimit = 12000;
                i++;
            }
        } else if (i == j++) {
            if (getDrfb() > drfbPos1 + 300) {
                drfbPid.target = getDrfb();
                drfbPidRunning = true;
            } else {
                setDrfb(12000);
            }
            if (getDrfb() > drfbMinClaw) clawPid.target = claw180;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                drfbPid.doneTime = BIL;
                drfbPid.target = getDrfb();
                drfbPidRunning = true;
                t0 = millis();
                i++;
            }

        } else if (i == j++) {
            setDL(3000);
            setDR(3000);
            if (millis() - t0 > 800) {
                i++;
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                odometry.setX(0);
                odometry.setX(0);
                odometry.setA(-PI / 2);
                targetPos.x = 0;
                targetPos.y = 1.5;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                i++;
            }
        } else if (i == j++) {
            setDrfb(-12000);
            if (getDrfb() < drfbPos1 + 100) {
                targetPos.y = 15;
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos1;
            if (pidDrive(targetPos, driveT)) {
                setDL(0);
                setDR(0);
                i++;
            }
        } else {
            stopMotors();
        }
        pidClaw();
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        setIntake(is);
        setFlywheel(0);
        if (i != prevI) {
            for (int w = 0; w < 15; w++) cout << endl;
        }
        prevI = i;
        if (millis() - lastT > 100) {
            printDrivePidValues();
            lastT = millis();
        }
        delay(10);
    }
}
void autonomous() {
    setupAuton();
    auton1(true);
    stopMotors();
}
