// 轻量四元数旋转库 (STM32 版本)

#pragma once

#include "vector.h"
#include <math.h>

class Quaternion {
public:
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // 从轴角构造
    static Quaternion fromAxisAngle(const Vector& axis, float angle) {
        float half = angle * 0.5f;
        float sin2 = sinf(half);
        float cos2 = cosf(half);
        float sinNorm = (axis.norm() > 1e-9f) ? sin2 / axis.norm() : 0;
        return Quaternion(cos2, axis.x * sinNorm, axis.y * sinNorm, axis.z * sinNorm);
    }

    // 从旋转向量 (轴 * 角度) 构造
    static Quaternion fromRotationVector(const Vector& rotation) {
        if (rotation.zero()) return Quaternion();
        return fromAxisAngle(rotation, rotation.norm());
    }

    // 从欧拉角 (roll, pitch, yaw) 构造
    static Quaternion fromEuler(const Vector& euler) {
        float cx = cosf(euler.x * 0.5f);
        float cy = cosf(euler.y * 0.5f);
        float cz = cosf(euler.z * 0.5f);
        float sx = sinf(euler.x * 0.5f);
        float sy = sinf(euler.y * 0.5f);
        float sz = sinf(euler.z * 0.5f);

        return Quaternion(
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz);
    }

    bool finite() const { return isfinite(w) && isfinite(x) && isfinite(y) && isfinite(z); }
    bool valid() const   { return finite(); }
    bool invalid() const { return !valid(); }

    void invalidate() { w = x = y = z = NAN; }

    float norm() const { return sqrtf(w * w + x * x + y * y + z * z); }

    void normalize() {
        float n = norm();
        if (n > 1e-9f) { w /= n; x /= n; y /= n; z /= n; }
    }

    // 四元数乘法 (a * b)
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y + y * q.w + z * q.x - x * q.z,
            w * q.z + z * q.w + x * q.y - y * q.x);
    }

    bool operator==(const Quaternion& q) const {
        return w == q.w && x == q.x && y == q.y && z == q.z;
    }
    bool operator!=(const Quaternion& q) const { return !(*this == q); }

    // 共轭 (逆四元数)
    Quaternion inversed() const {
        float n2Inv = 1.0f / (w * w + x * x + y * y + z * z);
        return Quaternion(w * n2Inv, -x * n2Inv, -y * n2Inv, -z * n2Inv);
    }

    // 四元数旋转
    static Quaternion rotate(const Quaternion& a, const Quaternion& b, bool doNormalize = true) {
        Quaternion r = a * b;
        if (doNormalize) r.normalize();
        return r;
    }

    // 用四元数旋转向量
    static Vector rotateVector(const Vector& v, const Quaternion& q) {
        // q^-1 * v * q
        Quaternion qv(0, v.x, v.y, v.z);
        Quaternion res = q.inversed() * qv * q;
        return Vector(res.x, res.y, res.z);
    }

    // 转欧拉角 (roll, pitch, yaw)
    Vector toEuler() const {
        Vector euler;
        float sqx = x * x, sqy = y * y, sqz = z * z, sqw = w * w;
        float sarg = -2.0f * (x * z - w * y) / (sqx + sqy + sqz + sqw);

        if (sarg <= -0.99999f) {
            euler.x = 0;
            euler.y = -0.5f * PI;
            euler.z = -2.0f * atan2f(y, x);
        } else if (sarg >= 0.99999f) {
            euler.x = 0;
            euler.y =  0.5f * PI;
            euler.z =  2.0f * atan2f(y, x);
        } else {
            euler.x = atan2f(2.0f * (y * z + w * x), sqw - sqx - sqy + sqz);
            euler.y = asinf(sarg);
            euler.z = atan2f(2.0f * (x * y + w * z), sqw + sqx - sqy - sqz);
        }
        return euler;
    }

    float getYaw()   const { return toEuler().z; }
    float getRoll()  const { return toEuler().x; }
    float getPitch() const { return toEuler().y; }
};
