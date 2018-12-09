#ifndef MOTOR_SAVER_H
#define MOTOR_SAVER_H

class MotorSaver {
   private:
    int prevEncVal, speedsLen, maxSpeed, maxPwr;
    int *speeds;
    int *pwrs;
    MotorSaver();

   public:
    MotorSaver(int iterations);
    ~MotorSaver();
    int getPwr(int inputPwr, int encoderValue);
};

#endif