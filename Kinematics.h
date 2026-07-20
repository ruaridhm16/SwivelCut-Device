// Pure joint-space / Cartesian math for the two-link arm. No hardware state.
#pragma once

// Wraps an angle to (-180, 180].
float normalizeJointDegrees(float degreesValue);

// Signed shortest distance from currentDeg to targetDeg, wrapped to (-180, 180].
float shortestJointDelta(float targetDeg, float currentDeg);

// The representation of targetDeg nearest referenceDeg on the continuous
// (unwrapped) joint axis, so callers can command "the short way around".
float equivalentJointTargetNear(float targetDeg, float referenceDeg);

bool angleInRange(float j1Deg, float j2Deg);

void forwardKinematicsForLink2(
    float j1Deg, float j2Deg, float link2Mm, float &x, float &y);
void forwardKinematics(float j1Deg, float j2Deg, float &x, float &y);

bool inverseKinematicsForLink2(
    float x, float y, bool elbowDown, float link2Mm,
    float &j1Deg, float &j2Deg);
bool inverseKinematics(
    float x, float y, bool elbowDown, float &j1Deg, float &j2Deg);
