// 低通滤波器 (模板类)

#pragma once

template <typename T>
class LowPassFilter {
public:
    float alpha;
    T output;

    LowPassFilter(float a = 1.0f) : alpha(a) {}

    T update(const T& input) {
        if (!init) {
            init = true;
            output = input;
            return output;
        }
        output += alpha * (input - output);
        return output;
    }

    void reset() { init = false; }

    void setCutOffFrequency(float cutOffFreq, float dt) {
        alpha = 1.0f - expf(-2.0f * PI * cutOffFreq * dt);
    }

private:
    bool init = false;
};
