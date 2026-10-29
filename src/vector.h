// 轻量三维向量库 (STM32 版本)

#pragma once

#include <math.h>

class Vector {
public:
    float x, y, z;

    Vector() : x(0), y(0), z(0) {}
    Vector(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    bool zero() const { return x == 0 && y == 0 && z == 0; }

    bool finite() const { return isfinite(x) && isfinite(y) && isfinite(z); }
    bool valid() const   { return finite(); }
    bool invalid() const { return !valid(); }

    void invalidate() { x = y = z = NAN; }

    float norm() const { return sqrtf(x * x + y * y + z * z); }

    void normalize() {
        float n = norm();
        if (n > 1e-9f) { x /= n; y /= n; z /= n; }
    }

    Vector operator+(const float b) const { return Vector(x + b, y + b, z + b); }
    Vector operator-(const float b) const { return Vector(x - b, y - b, z - b); }
    Vector operator*(const float b) const { return Vector(x * b, y * b, z * b); }
    Vector operator/(const float b) const { return Vector(x / b, y / b, z / b); }

    Vector operator+(const Vector& b) const { return Vector(x + b.x, y + b.y, z + b.z); }
    Vector operator-(const Vector& b) const { return Vector(x - b.x, y - b.y, z - b.z); }

    Vector& operator+=(const Vector& b) { x += b.x; y += b.y; z += b.z; return *this; }
    Vector& operator-=(const Vector& b) { x -= b.x; y -= b.y; z -= b.z; return *this; }

    // 逐元素乘除
    Vector operator*(const Vector& b) const { return Vector(x * b.x, y * b.y, z * b.z); }
    Vector operator/(const Vector& b) const { return Vector(x / b.x, y / b.y, z / b.z); }

    bool operator==(const Vector& b) const { return x == b.x && y == b.y && z == b.z; }
    bool operator!=(const Vector& b) const { return !(*this == b); }

    static float dot(const Vector& a, const Vector& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vector cross(const Vector& a, const Vector& b) {
        return Vector(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
    }

    static float angleBetween(const Vector& a, const Vector& b) {
        float d = dot(a, b) / (a.norm() * b.norm());
        if (d > 1.0f) d = 1.0f;
        if (d < -1.0f) d = -1.0f;
        return acosf(d);
    }

    // 从 a 旋转到 b 的旋转向量 (轴 * 角度)
    static Vector rotationVectorBetween(const Vector& a, const Vector& b) {
        float an = a.norm();
        float bn = b.norm();
        if (an < 1e-6f || bn < 1e-6f) return Vector(0, 0, 0);

        Vector direction = cross(a, b);
        if (direction.norm() < 1e-6f) {
            // 两向量平行
            if (dot(a, b) > 0) return Vector(0, 0, 0);
            // 方向相反, 构造一个垂向量
            Vector perp = cross(a, Vector(1, 0, 0));
            if (perp.norm() < 1e-6f) perp = cross(a, Vector(0, 1, 0));
            perp.normalize();
            return perp * PI;
        }
        direction.normalize();
        float angle = angleBetween(a, b);
        return direction * angle;
    }
};

inline Vector operator*(const float a, const Vector& b) { return b * a; }
inline Vector operator+(const float a, const Vector& b) { return b + a; }
inline Vector operator-(const float a, const Vector& b) { return Vector(a - b.x, a - b.y, a - b.z); }
