#ifndef MOTOR_SAVER_H
#define MOTOR_SAVER_H

class MotorSaver {
   private:
    int prevEncVal, speedsLen, maxSpeed, maxPwr, sumSpeed, sumPwr;
    int *speeds;
    int *pwrs;
    MotorSaver();

   public:
    MotorSaver(int iterations);
    ~MotorSaver();
    int getPwr(int inputPwr, int encoderValue);
    int getSumSpeed();
    int getSumPwr();
    int getMaxSpeed();
    int getMaxPwr();
    bool isFaster(double d);
   bool isPwr(double d);
};

#endif
