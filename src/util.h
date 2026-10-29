// STM32 版本的工具函数
// 移除了 ESP32_NOW_Serial 依赖

#pragma once

#include <math.h>

const float ONE_G = 9.80665f;
extern float t;

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline bool invalid(float x) {
    return !isfinite(x);
}

inline bool valid(float x) {
    return isfinite(x);
}

inline bool floatEquals(float a, float b, float epsilon = 0) {
    if (isnan(a) && isnan(b)) return true;
    if (a == b) return true;
    return fabsf(a - b) <= epsilon;
}

// 角度归一化到 [-PI, PI)
inline float wrapAngle(float angle) {
    angle = fmodf(angle, 2 * PI);
    if (angle > PI) {
        angle -= 2 * PI;
    } else if (angle < -PI) {
        angle += 2 * PI;
    }
    return angle;
}

// 限幅 (STM32duino 也有 constrain, 这里冗余安全)
#ifndef constrain
template<typename T>
inline T constrain(T x, T lo, T hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
#endif

// 频率限频器
class Rate {
public:
    float rate;
    float last = -INFINITY;
    Rate(float rate) : rate(rate) {}

    operator bool() {
        if (t - last >= 1.0f / rate) {
            last = t;
            return true;
        }
        return false;
    }
};

// 布尔信号延时滤波器: 确保信号持续 delay 秒后才生效
class Delay {
public:
    float delay;
    float start = NAN;
    Delay(float delay) : delay(delay) {}

    bool update(bool on) {
        if (!on) {
            start = NAN;
            return false;
        } else if (isnan(start)) {
            start = t;
        }
        return t - start >= delay;
    }
};
