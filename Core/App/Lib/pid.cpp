/*
 * pid.cpp
 * Created on: 2023 4/14
 * Author: G4T1PR0
 */

#include "pid.hpp"

// Public

PID::PID() {
    _prev_time = 10;
    _kp = 0;
    _ki = 0;
    _kd = 0;
    _error = 0;
    _prev_error = 0;
    _integral = 0;
    _prev_integral = 0;
    _derivative = 0;
    _output = 0;
}

void PID::setPID(float p, float i, float d) {
    _kp = p;
    _ki = i;
    _kd = d;
}

void PID::setP(float p) {
    _kp = p;
}

void PID::setI(float i) {
    _ki = i;
}

void PID::setD(float d) {
    _kd = d;
}

void PID::setProcessTime(unsigned int time) {
    _prev_time = time;
}

float PID::getP() {
    return _kp;
}

float PID::getI() {
    return _ki;
}

float PID::getD() {
    return _kd;
}

float PID::getError() {
    return _error;
}

float PID::getIntegral() {
    return _integral;
}

float PID::getDerivative() {
    return _derivative;
}

float PID::getOutput() {
    return _output;
}

float PID::update(float target, float feedback) {
    _error = target - feedback;
    _integral += _error;
    _derivative = (_error - _prev_error) / (_prev_time) * 0.001;
    _prev_error = _error;
    _output = _kp * _error + _ki * _integral + _kd * _derivative;
    return _output;
}