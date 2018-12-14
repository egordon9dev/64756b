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
using std::endl; /*
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
 }*/
void auton2(bool leftSide) {
    setupAuton();
    int sideSign = leftSide ? 1 : -1;
    odometry.setXAxisDir(sideSign);
    // odometry.setRotationDir(sideSign);
    int i = 0;
    int k = 0;
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(-4);
    double targetAngle = -PI / 2;
    const int driveT = 800;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false;
    int prevITime = millis();
    int timeBetweenI = 4000;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;
    while (!ctlr.get_digital(DIGITAL_B)) {
		if(i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
		if(millis() - autonT0 > 1500000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) break;
        int j = 0;
        odometry.update();
        if (i == j++) {
            t0 = millis();
            pidDriveInit(Point(0, 45), driveT);
            i++;
            k = 0;
        } else if (i == j++) {  // grab ball from under cap 1
            // deploy claw
            if (k == 0) {
                setDrfb(12000);
                drfbPidRunning = false;
                if (getDrfb() > drfb18Max && millis() - autonT0 > 300) k++;
            } else {  // lower lift
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            flywheelPid.target = 2.9;
            setClaw(0);
            is = IntakeState::FRONT;
            if (pidDrive()) {
                i++;
                pidFollowArcInit(Point(0, 45), Point(14 * sideSign, 45), 5, sideSign, 1000);
            }
        } else if (i == j++) {  // arc twd cap 2
			printf("arc twd cap 2 ");
            printing = false;
            printArcData();
            if (pidDriveArc()) {
                printing = true;
				pidDriveInit(Point(14*sideSign, 51), driveT);
                i++;
            }
		} else if(i == j++) {
			printf("drv twd cap 2");
			if(pidDrive()) {
				t0 = millis();
				i++;
			}
        } else if (i == j++) { // lift cap
			printf("liftCap ");
            setDL(0);
            setDR(0);
			drfbPidRunning = false;
			setDrfb(12000);
            if (getDrfb() > drfb18Max) {
				drfbPidRunning = true;
                drfbPid.target = drfbPos1 + 250;
                pidDriveArcInit(Point(14 * sideSign, 51), Point(-2 * sideSign, 21), 60, -sideSign, driveT);
                i++;
            }
        } else if (i == j++) {
			printf("arc twd pipe ");
            if (getDrfb() > drfbMinClaw) clawPid.target = clawPos0;
            if (pidDriveArc()) {  // arc twd pipe
                t0 = BIL;
                i++;
            }
        } else if (i == j++) {  // funnel drive
			printf("funnel drive ");
            if (leftSide) {
                setDL(8000);
                setDR(5000);
            } else {
                setDL(5000);
                setDR(8000);
            }
            if (!dlSaver.isFaster(0.1) && !drSaver.isFaster(0.1) && (dlSaver.isPwr(0.25) || drSaver.isPwr(0.25))) {
                if (t0 > (int)millis()) t0 = millis();
                if (millis() - t0 > 300) {
                    i++;
                    odometry.setX(0);
                    odometry.setY(0);
                    odometry.setA(leftSide ? 0 : PI);
                    pidDriveInit(Point(-sideSign * 3, 0), driveT);
                }
            }
        } else if (i == j++) {
            if (pidDrive()) {
                drfbPid.target = drfbPos0;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            drfbPidRunning = false;
            setDrfb(-12000);
            if (getDrfb() < drfbPos1 + 50 || millis() - t0 > 800) {
                drfbPid.target = drfbPos1;
                drfbPidRunning = true;
                t0 = millis();
                pidFollowArcInit(Point(-sideSign * 3, 0), Point(-sideSign * 20, -sideSign * 9), 50, sideSign, driveT);
                i++;
            }
        } else if (i == j++) {  // s-curve to flywheel pos
            if (millis() - t0 > 300) drfbPid.target = drfbPos0;
            if (pidDriveArc()) {
                pidDriveArcInit(Point(-sideSign * 20, -sideSign * 9), Point(-sideSign * 40, -sideSign * 18), 50, -sideSign, driveT);
                i++;
            }
        } else if (i == j++) {  // s-cruve to flywheel pos
            if (pidDriveArc()) {
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // shoot
            is = IntakeState::ALL;
            if (millis() - t0 > 1000) {
				t0 = millis();
				flywheelPid.target = 2.7;
				i++;
			}
		} else if(i == j++) {
			is = IntakeState::NONE;
			if(millis() - t0 > 800) {
				t0 = millis();
				i++;
			}
		} else if(i == j++) {
			is = IntakeState::ALL;
			if(millis() - t0 > 1000) {
				i++;
			}
        } else {
			printing = false;
			if(i == 12345) printf("\n\nAUTON TIMEOUT\n");
			i = 99999;
            stopMotors();
        }
        if (clawPidRunning) pidClaw();
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        setIntake(is);
        if (i != prevI) {
            for (int w = 0; w < 15; w++) cout << endl;
        }
        if (millis() - lastT > 100 && printing) {
			printf("t%d ", millis()-autonT0);
            printDrivePidValues();
            lastT = millis();
        }
        delay(10);
    }
    stopMotors();
}
void auton3() {
	
}
void autonomous() {
    setupAuton();
    auton2(true);
    stopMotors();
}
