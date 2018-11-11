#ifndef SETUP_H
#define SETUP_H
#include "main.h"
#include "pid.hpp"
#define TICKS_TO_DEG 0.4
#define INT_MAX 2147483647
extern pros::Motor mtr5, mtr6, mtr7, mtr8, mtr9, mtr10, mtr11, mtr12;
extern pros::Controller ctlr;
extern pros::ADIPotentiometer* drfbPot;
extern pros::ADILineSensor* ballSens;
extern const int drfbMinPos, drfbMaxPos, drfbPos1, drfbPos2, drfbMinClaw;
extern const int ctlrIdxLeft, ctlrIdxUp, ctlrIdxRight, ctlrIdxDown, ctlrIdxY, ctlrIdxX, ctlrIdxA, ctlrIdxB, ctlrIdxL1, ctlrIdxL2, ctlrIdxR1, ctlrIdxR2;
enum class IntakeState { ALL, FRONT, NONE };
int clamp(int n, int min, int max);

//------- Misc ----------
// returns prevClicks, curClicks, DblClicks
bool** getAllClicks();
void printAllClicks(int line, bool** allClicks);

// -------- Drive --------
void setDR(int n);
void setDL(int n);

//----------- Intake ------
void intakeNone();
void intakeFront();
void intakeAll();
void setIntake(IntakeState is);
int getBallSens();

//----------- DRFB functions ---------
void setDrfb(int n);
double getDrfb();
bool pidDrfb(double pos, int wait);
void pidDrfb();
//---------- Claw functions --------
void setClaw(int n);
double getClaw();
bool pidClawAbsolute(double a, int wait);
bool pidClaw(double a, int wait, int id);
//--------- Flywheel functions --------
void setFlywheel(int n);
double getFlywheel();
bool pidFlywheel(double speed);
bool pidFlywheel();
void setup();
#endif
