// 参数存储相关定义 (抽到独立头, 避免 Arduino .ino 合并顺序问题)
#ifndef FLIX_PARAMETER_H
#define FLIX_PARAMETER_H

#include <Arduino.h>

// 每个参数存储格式: [16B name][4B value as float] — 共 20 字节
static const uint16_t PARAM_NAME_LEN = 16;

struct Parameter {
    const char* name;
    bool isInteger;
    union { float* f; int* i; };
    float initial;
    float cache;
    void (*callback)();

    Parameter() : name(nullptr), isInteger(false), f(nullptr),
                  initial(0), cache(0), callback(nullptr) {}
    Parameter(const char* n, float* var, void (*cb)() = nullptr)
        : name(n), isInteger(false), f(var), initial(0), cache(0), callback(cb) {}
    Parameter(const char* n, int* var, void (*cb)() = nullptr)
        : name(n), isInteger(true), i(var), initial(0), cache(0), callback(cb) {}
};

#endif // FLIX_PARAMETER_H
