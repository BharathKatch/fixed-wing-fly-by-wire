#pragma once

#include "MathTypes.h"

namespace MathUtils {

float quatMagnitude(const Quaternion& q);
Quaternion normalizeQuat(const Quaternion& q);

Quaternion conjugateQuat(const Quaternion& q);
Quaternion multiplyQuat(const Quaternion& q1, const Quaternion& q2);

Vector3 rotateVector(const Quaternion& q, const Vector3& v);

float vectorMagnitude(const Vector3& v);
Vector3 normalizeVector(const Vector3& v);

float dotVector(const Vector3& a, const Vector3& b);
Vector3 crossVector(const Vector3& a, const Vector3& b);

Vector3 addVector(const Vector3& a, const Vector3& b);
Vector3 subtractVector(const Vector3& a, const Vector3& b);
Vector3 scaleVector(const Vector3& v, float scalar);

}