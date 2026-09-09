#include "MathUtils.h"

#include <cmath>

namespace MathUtils {

float quatMagnitude(const Quaternion& q) {
    return sqrtf(
        q.w * q.w +
        q.x * q.x +
        q.y * q.y +
        q.z * q.z
    );
}

Quaternion normalizeQuat(const Quaternion& q) {
    float magnitude = quatMagnitude(q);

    if (magnitude <= 0.0f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    return {
        q.w / magnitude,
        q.x / magnitude,
        q.y / magnitude,
        q.z / magnitude
    };
}

Quaternion conjugateQuat(const Quaternion& q) {
    return {
        q.w,
        -q.x,
        -q.y,
        -q.z
    };
}

Quaternion multiplyQuat(const Quaternion& q1, const Quaternion& q2) {
    return {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,

        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,

        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,

        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
}

Vector3 rotateVector(const Quaternion& q, const Vector3& v) {
    Quaternion qNorm = normalizeQuat(q);

    Quaternion vQuat = {
        0.0f,
        v.x,
        v.y,
        v.z
    };

    Quaternion rotated = multiplyQuat(
        multiplyQuat(qNorm, vQuat),
        conjugateQuat(qNorm)
    );

    return {
        rotated.x,
        rotated.y,
        rotated.z
    };
}

float vectorMagnitude(const Vector3& v) {
    return sqrtf(
        v.x * v.x +
        v.y * v.y +
        v.z * v.z
    );
}

Vector3 normalizeVector(const Vector3& v) {
    float magnitude = vectorMagnitude(v);

    if (magnitude <= 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {
        v.x / magnitude,
        v.y / magnitude,
        v.z / magnitude
    };
}

float dotVector(const Vector3& a, const Vector3& b) {
    return (
        a.x * b.x +
        a.y * b.y +
        a.z * b.z
    );
}

Vector3 crossVector(const Vector3& a, const Vector3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vector3 addVector(const Vector3& a, const Vector3& b) {
    return {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

Vector3 subtractVector(const Vector3& a, const Vector3& b) {
    return {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}

Vector3 scaleVector(const Vector3& v, float scalar) {
    return {
        v.x * scalar,
        v.y * scalar,
        v.z * scalar
    };
}

}