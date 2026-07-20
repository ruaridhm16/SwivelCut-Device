#include "Kinematics.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

float normalizeJointDegrees(float degreesValue) {
  while (degreesValue > 180.0f) degreesValue -= 360.0f;
  while (degreesValue < -180.0f) degreesValue += 360.0f;
  return degreesValue;
}

float shortestJointDelta(float targetDeg, float currentDeg) {
  return normalizeJointDegrees(targetDeg - currentDeg);
}

float equivalentJointTargetNear(float targetDeg, float referenceDeg) {
  return referenceDeg + shortestJointDelta(targetDeg, referenceDeg);
}

bool angleInRange(float j1Deg, float j2Deg) {
  return j1Deg >= J1_MIN_DEG && j1Deg <= J1_MAX_DEG &&
         j2Deg >= J2_MIN_DEG && j2Deg <= J2_MAX_DEG;
}

void forwardKinematicsForLink2(
    float j1Deg, float j2Deg, float link2Mm, float &x, float &y) {
  const float t1 = radians(j1Deg);
  const float t2 = radians(j2Deg);
  y = LINK_1_MM * cosf(t1) + link2Mm * cosf(t2 - t1);
  x = -LINK_1_MM * sinf(t1) + link2Mm * sinf(t2 - t1);
}

void forwardKinematics(float j1Deg, float j2Deg, float &x, float &y) {
  forwardKinematicsForLink2(j1Deg, j2Deg, LINK_2_MM, x, y);
}

bool inverseKinematicsForLink2(
    float x, float y, bool elbowDown, float link2Mm,
    float &j1Deg, float &j2Deg) {
  float c2 = (x * x + y * y - LINK_1_MM * LINK_1_MM -
              link2Mm * link2Mm) / (2.0f * LINK_1_MM * link2Mm);
  if (c2 < -1.00001f || c2 > 1.00001f) return false;
  c2 = constrain(c2, -1.0f, 1.0f);
  float s2 = sqrtf(max(0.0f, 1.0f - c2 * c2));
  if (elbowDown) s2 = -s2;
  const float t2 = atan2f(s2, c2);
  const float t1 =
      atan2f(x, y) - atan2f(link2Mm * s2, LINK_1_MM + link2Mm * c2);
  j1Deg = -degrees(t1);
  j2Deg = degrees(t2);
  return angleInRange(j1Deg, j2Deg);
}

bool inverseKinematics(
    float x, float y, bool elbowDown, float &j1Deg, float &j2Deg) {
  return inverseKinematicsForLink2(x, y, elbowDown, LINK_2_MM, j1Deg, j2Deg);
}
