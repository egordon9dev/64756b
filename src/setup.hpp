#ifndef SETUP_H
#define SETUP_H
#include "main.h"
#include "pid.hpp"
#define TICKS_TO_DEG 0.4
extern pros::Motor mtr5, mtr6, mtr7, mtr8, mtr9, mtr10, mtr11, mtr12;
extern pros::Controller ctlr;
extern pros::ADIPotentiometer* drfbPot;
extern pros::ADILineSensor* ballSens;
extern const int drfbMinPos, drfbMaxPos, drfbPos0, drfbPos1, drfbPos2, drfbMinClaw, claw180;
extern const int ctlrIdxLeft, ctlrIdxUp, ctlrIdxRight, ctlrIdxDown, ctlrIdxY, ctlrIdxX, ctlrIdxA, ctlrIdxB, ctlrIdxL1, ctlrIdxL2, ctlrIdxR1, ctlrIdxR2;
extern const int BIL, MIL;
extern const int dblClickTime;
extern const double PI;
extern const double ticksPerInch;
enum class IntakeState { ALL, FRONT, NONE };
int clamp(int n, int min, int max);
double clamp(double n, double min, double max);

//------- Misc ----------
// returns prevClicks, curClicks, DblClicks
bool** getAllClicks();
void printAllClicks(int line, bool** allClicks);
void printPidValues();
void stopMotors();
Point polarToRect(double mag, double angle);

// -------- Drive --------
void setDR(int n);
void setDL(int n);
double getDR();
double getDL();
int getDLVoltage();
int getDRVoltage();
void printDrivePidValues();

//----------- Intake ------
void intakeNone();
void intakeFront();
void intakeAll();
void setIntake(IntakeState is);
int getBallSens();
bool isBallIn();

//----------- DRFB functions ---------
void setDrfb(int n);
void setDrfbParams(bool auton);
int getDrfb();
bool pidDrfb(double pos, int wait);
void pidDrfb();
//---------- Claw functions --------
void setClaw(int n);
double getClaw();
bool pidClaw(double a, int wait);
void pidClaw();
//--------- Flywheel functions --------
void setFlywheel(int n);
double getFlywheel();
bool pidFlywheel(double speed);
bool pidFlywheel();
void setupAuton();
void setupOpCtrl();
#endif
