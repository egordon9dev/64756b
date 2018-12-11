#include "MotorSaver.hpp"
#include "setup.hpp"
MotorSaver::MotorSaver(){};
MotorSaver::MotorSaver(int iterations) {
    speedsLen = iterations;
    speeds = new int[iterations];
    pwrs = new int[iterations];
    maxSpeed = 30 * speedsLen;
    maxPwr = 12000 * speedsLen;
}
MotorSaver::~MotorSaver() {
    delete[] speeds;
    delete[] pwrs;
}
int MotorSaver::getPwr(int inputPwr, int encoderValue) {
    // record array of speeds
    static bool first = true;
    if (first) {
        for (int i = 0; i < speedsLen; i++) speeds[i] = pwrs[i] = 0;
    }
    first = false;
    for (int i = 0; i < speedsLen - 1; i++) {
        speeds[i] = speeds[i + 1];
        pwrs[i] = pwrs[i + 1];
    }
    speeds[speedsLen - 1] = abs(encoderValue - prevEncVal);
    prevEncVal = encoderValue;
    pwrs[speedsLen - 1] = abs(inputPwr);

    // limit stall torque to protect motor
    sumSpeed = 0, sumPwr = 0;
    for (int i = 0; i < speedsLen; i++) {
        sumSpeed += speeds[i];
        sumPwr += pwrs[i];
    }
    if (sumSpeed < maxSpeed * 0.1 && sumPwr > maxPwr * 0.4) { inputPwr = clamp(inputPwr, -2000, 2000); }
    if (sumSpeed < maxSpeed * 0.02 && sumPwr > maxPwr * 0.25) { inputPwr = 0; }
    return clamp(inputPwr, -12000, 12000);
}
int MotorSaver::getSumSpeed() { return sumSpeed; }
int MotorSaver::getSumPwr() { return sumPwr; }
int MotorSaver::getMaxSpeed() { return maxSpeed; }
int MotorSaver::getMaxPwr() { return maxPwr; }
bool MotorSaver::isFaster(double d) { return sumSpeed > maxSpeed * d; }