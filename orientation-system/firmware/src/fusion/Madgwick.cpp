#include "Madgwick.h"

#include "Config.h"

#include <math.h>

static constexpr float kRadToDeg = 57.295779513082320876798154814105f;

using namespace core::config;

void Madgwick::begin() {
  q0 = 1.0f;
  q1 = q2 = q3 = 0.0f;
}

void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float beta,
                         float dt) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  const float _2q0 = 2.0f * q0;
  const float _2q1 = 2.0f * q1;
  const float _2q2 = 2.0f * q2;
  const float _2q3 = 2.0f * q3;
  const float _4q0 = 4.0f * q0;
  const float _4q1 = 4.0f * q1;
  const float _4q2 = 4.0f * q2;
  const float _8q1 = 8.0f * q1;
  const float _8q2 = 8.0f * q2;
  const float q0q0 = q0 * q0;
  const float q1q1 = q1 * q1;
  const float q2q2 = q2 * q2;
  const float q3q3 = q3 * q3;

  float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
  float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

  norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (norm < 1e-9f) return;
  s0 /= norm;
  s1 /= norm;
  s2 /= norm;
  s3 /= norm;

  const float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  const float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  const float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  const float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my,
                      float mz, float beta, float dt) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm < 1e-6f) {
    updateIMU(gx, gy, gz, ax, ay, az, beta, dt);
    return;
  }
  mx /= norm;
  my /= norm;
  mz /= norm;

  const float _2q0mx = 2.0f * q0 * mx;
  const float _2q0my = 2.0f * q0 * my;
  const float _2q0mz = 2.0f * q0 * mz;
  const float _2q1mx = 2.0f * q1 * mx;
  const float _2q0 = 2.0f * q0;
  const float _2q1 = 2.0f * q1;
  const float _2q2 = 2.0f * q2;
  const float _2q3 = 2.0f * q3;
  const float q0q0 = q0 * q0;
  const float q0q1 = q0 * q1;
  const float q0q2 = q0 * q2;
  const float q0q3 = q0 * q3;
  const float q1q1 = q1 * q1;
  const float q1q2 = q1 * q2;
  const float q1q3 = q1 * q3;
  const float q2q2 = q2 * q2;
  const float q2q3 = q2 * q3;
  const float q3q3 = q3 * q3;

  const float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 -
                   mx * q2q2 - mx * q3q3;
  const float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 +
                   _2q2 * mz * q3 - my * q3q3;
  const float _2bx = sqrtf(hx * hx + hy * hy);
  const float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 -
                     mz * q2q2 + mz * q3q3;
  const float _4bx = 2.0f * _2bx;
  const float _4bz = 2.0f * _2bz;

  float s0 = -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2.0f * (q0q1 + q2q3) - ay) -
             _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s1 = _2q3 * (2.0f * (q1q3 - q0q2) - ax) + _2q0 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q1 * (2.0f * (0.5f - q1q1 - q2q2) - az) +
             _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s2 = -_2q0 * (2.0f * (q1q3 - q0q2) - ax) + _2q3 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q2 * (2.0f * (0.5f - q1q1 - q2q2) - az) +
             (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s3 = _2q1 * (2.0f * (q1q3 - q0q2) - ax) + _2q2 * (2.0f * (q0q1 + q2q3) - ay) +
             (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

  norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (norm < 1e-9f) return;
  s0 /= norm;
  s1 /= norm;
  s2 /= norm;
  s3 /= norm;

  const float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  const float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  const float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  const float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void Madgwick::eulerDeg(float &roll, float &pitch, float &yaw) const {
  roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * kRadToDeg;
  pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (q0 * q2 - q3 * q1)))) * kRadToDeg;
  yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * kRadToDeg;
  if (yaw < 0.0f)    yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

void Madgwick::copyQuat(float out[4]) const {
  out[0] = q0;
  out[1] = q1;
  out[2] = q2;
  out[3] = q3;
}
