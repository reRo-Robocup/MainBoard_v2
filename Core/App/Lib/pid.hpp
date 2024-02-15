/*
 * pid.hpp
 * Created on: 2023 4/14
 * Author: G4T1PR0
 */

#pragma once

class PID {
   public:
    PID();
    void setPID(float p, float i, float d);
    void setP(float p);
    void setI(float i);
    void setD(float d);
    void setProcessTime(unsigned int);
    float getP();
    float getI();
    float getD();
    float getError();
    float getIntegral();
    float getDerivative();
    float getOutput();

    float update(float target, float feedback);

   private:
    unsigned long long _prev_time;
    float _kp, _ki, _kd;
    float _error, _prev_error, _integral, _prev_integral, _derivative, _output;
};
