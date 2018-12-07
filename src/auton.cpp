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
    // odometry.setXAxisDir(sideSign);
    // odometry.setRotationDir(sideSign);
    int i = 0;
    odometry.setA(-PI / 2);
    Point targetPos(0, 43);
    double targetAngle = -PI / 2;
    const int driveT = 500;
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
            flywheelPid.target = 2.9;
            drfbPid.target = 1800;
            is = IntakeState::FRONT;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x = -28 * sideSign;
                targetPos.y = 28;
                arcRadius = (targetPos - odometry.getPos()).mag() + 3;
                i++;
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos0 + 50;
            double distanceToTarget = (targetPos - odometry.getPos()).mag();
            if (arcRadius < distanceToTarget + 1) arcRadius = distanceToTarget + 1;
            if (pidDriveArc(targetPos, arcRadius, -sideSign, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetAngle = odometry.getA() + sideSign * PI / 2;
                g_pidTurnLimit = 6000;
                i++;
            }
        } else if (i == j++) {
            if (pidTurn(targetAngle, driveT)) {
                g_pidTurnLimit = 12000;
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x = -14 * sideSign;
                targetPos.y = 12;
                arcRadius = (targetPos - odometry.getPos()).mag() + 1;
                i++;
            }
        } else if (i == j++) {
            double distanceToTarget = (targetPos - odometry.getPos()).mag();
            if (arcRadius < distanceToTarget + 1) arcRadius = distanceToTarget + 1;
            if (pidDriveArc(targetPos, arcRadius, sideSign, MIL)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x -= 8 * sideSign;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // SHOOT BALL 1
            setDL(0);
            setDR(0);
            is = IntakeState::ALL;
            if (millis() - t0 > 700) {
                is = IntakeState::NONE;
                targetPos.x -= 14 * sideSign;
                flywheelPid.doneTime = BIL;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x += 33 * sideSign;
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
                targetPos.x += 11 * sideSign;
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
                targetAngle -= PI / 4 * sideSign;
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
                if (leftSide) {
                    setDL(12000);
                    setDR(1000);
                } else {
                    setDR(12000);
                    setDL(1000);
                }
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
    // odometry.setRotationDir(sideSign);
    int i = 0;
	int k = 0;
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(0);
    Point targetPos(0, 45);
    double targetAngle = -PI / 2;
    const int driveT = 500;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = true;
    int prevITime = millis();
    int timeBetweenI = 4000;
	drivePid.doneTime = BIL;
	turnPid.doneTime = BIL;
    while (!ctlr.get_digital(DIGITAL_B)) {
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) break;
        int j = 0;
        odometry.update();
        if (i == j++) {
            t0 = millis();
            drivePid.doneTime = BIL;
            turnPid.doneTime = BIL;
            i++;
			k = 0;
        } else if (i == j++) {
			if(k == 0) {
                setDrfb(12000);
                drfbPidRunning = false;
				if (millis() - t0 > 500 || getDrfb() > drfbMinClaw - 50) k++;
			} else {
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            is = IntakeState::FRONT;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                arcRadius = (targetPos - odometry.getPos()).mag() + 20;
                targetPos.x = 7 * sideSign;
                targetPos.y = 20;
                i++;
            }
        } else if (i == j++) { // arc twd cap 2
            if (pidDriveArc(targetPos, arcRadius, sideSign, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x = 14 * sideSign;
                targetPos.y = 50;
                targetAngle = odometry.getA() + PI * 0.8 * -sideSign;
                i++;
            }
        } else if (i == j++) { // arc twd cap 2
            if (pidDriveArc(targetPos, arcRadius, sideSign, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.x = 7.5;
                targetPos.y = 16;
				drfbPidRunning = false;
                g_pidTurnLimit = 12000;
                i++;
            }
        } else if (i == j++) {
			//pick up cap
        } else if (i == j++) {
            drfbPid.target = drfbPos1 - 500;
            drfbPidRunning = true;
            if (getDrfb() > drfbMinClaw) clawPid.target = claw180;
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                t0 = millis();
                i++;
            }

        } else if (i == j++) {
            if (leftSide) {
                setDL(4000);
                setDR(6000);
            } else {
                setDR(4000);
                setDL(6000);
            }
            if (millis() - t0 > 1000) {
                i++;
                t0 = millis();
            }
        } else if (i == j++) {
            if (millis() - t0 < 300) {
                setDL(-8000);
                setDR(-8000);
            } else {
                setDL(0);
                setDR(0);
            }
            drfbPidRunning = false;
            setDrfb(12000);
            if (getDrfb() > 3000) {
                i++;
                t0 = millis();
                drfbPid.target = 3000;
                drfbPidRunning = true;
            }
        } else if (i == j++) {
            setDL(6000);
            setDR(9000);
            if (millis() - t0 > 1200) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                odometry.setX(0);
                odometry.setX(0);
                odometry.setA(-PI / 2);
                targetPos.x = 0;
                targetPos.y = 5;
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(targetPos, driveT)) {
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                i++;
            }
        } else if (i == j++) {
            drfbPidRunning = false;
            setDrfb(-12000);
            if (getDrfb() < drfbPos1 + 100) {
                targetPos.y = 18;
                i++;
            }
        } else if (i == j++) {
            drfbPidRunning = true;
            drfbPid.target = drfbPos1;
            if (pidDrive(targetPos, driveT)) {
                setDL(0);
                setDR(0);
                drfbPid.doneTime = BIL;
                drivePid.doneTime = BIL;
                turnPid.doneTime = BIL;
                targetPos.y += 12;
                i++;
            }
        } else if (i == j++) {
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0 + 80) {
                setDrfb(-12000);
                setDL(0);
                setDR(0);
            } else {
                setDrfb(-2000);
                pidDrive(targetPos, 999999);
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
        if (millis() - lastT > 100) {
			printf("%d", i);
			printf(": ");
            printDrivePidValues();
            lastT = millis();
        }
        delay(10);
    }
	stopMotors();
}
void autonomous() {
    setupAuton();
    auton2(true);
    stopMotors();
}
