// PID 控制器

#pragma once

#include "filter.h"

extern float t;   // 访问全局时间 (util.h 中声明)

class PID {
public:
    float p, i, d;
    float windup;
    float dtMax;

    float derivative = 0;
    float integral = 0;

    LowPassFilter<float> lpf; // 微分低通

    PID(float p_, float i_ = 0, float d_ = 0,
        float wu = INFINITY, float dAlpha = 1, float dMax = 0.1f) :
        p(p_), i(i_), d(d_), windup(wu), lpf(dAlpha), dtMax(dMax) {}

    float update(float error) {
        float dt = t - prevTime;

        if (dt > 0 && dt < dtMax) {
            integral += error * dt;
            derivative = lpf.update((error - prevError) / dt);
        } else {
            integral = 0;
            derivative = 0;
        }

        prevError = error;
        prevTime = t;

        // 积分限幅
        float iTerm = i * integral;
        if (iTerm >  windup) iTerm =  windup;
        if (iTerm < -windup) iTerm = -windup;

        return p * error + iTerm + d * derivative;
    }

    void reset() {
        prevError = NAN;
        prevTime  = NAN;
        integral  = 0;
        derivative = 0;
        lpf.reset();
    }

private:
    float prevError = NAN;
    float prevTime  = NAN;
};
