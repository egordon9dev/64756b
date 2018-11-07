#ifndef SETUP_H
#define SETUP_H
#include "main.h"
#include "pid.hpp"
#define TICKS_TO_DEG 0.4
#define INT_MAX 2147483647
extern pros::Motor mtr5, mtr6, mtr7, mtr8, mtr9, mtr10, mtr11, mtr12;
extern pros::Controller ctlr;
extern pros::ADIPotentiometer* drfbPot;
extern const int drfbMinPos, drfbMaxPos;

int clamp(int n, int min, int max);

void setDR(int n);
void setDL(int n);
void intakeNone();
void intakeFront();
void intakeAll();

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