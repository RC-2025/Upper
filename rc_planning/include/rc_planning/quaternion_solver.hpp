#pragma once
#include <sensor_msgs/msg/imu.hpp>

namespace robot {
namespace detail {

inline float inv_sqrt(float n) {
  volatile long i = 0;
  volatile float x = 0, y = 0;
  volatile const float f = 1.5F;
  x = n * 0.5F;
  y = n;
  i = *((long *)&y);
  i = 0x5f375a86 - (i >> 1);
  y = *((float *)&i);
  y = y * (f - (x * y * y));
  return y;
}

static volatile float twoKp = 1.0f; // 2 * proportional gain (Kp)
static volatile float twoKi = 0.0f; // 2 * integral gain (Ki)
static volatile float
    q0 = 1.0f,
    q1 = 0.0f, q2 = 0.0f,
    q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
static volatile float integralFBx = 0.0f, integralFBy = 0.0f,
                      integralFBz = 0.0f; // integral error terms scaled by Ki
constexpr float SAMPLING_FREQ = 20.f;
static void quaternion_solver(float gx, float gy, float gz, float ax, float ay,
                              float az, sensor_msgs::msg::Imu &imu_) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex *
                     (1.0f / SAMPLING_FREQ); // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / SAMPLING_FREQ)); // pre-multiply common factors
  gy *= (0.5f * (1.0f / SAMPLING_FREQ));
  gz *= (0.5f * (1.0f / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  // Normalise quaternion
  recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  imu_.orientation.w = q0;
  imu_.orientation.x = q1;
  imu_.orientation.y = q2;
  imu_.orientation.z = q3;
}

} // namespace detail
} // namespace robot