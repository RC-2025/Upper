#pragma once
#include <array>

namespace robot {
namespace ekf_config {
constexpr std::array<double, 36> odom_pose_covariance = {
    1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};

constexpr std::array<double, 36> odom_pose_covariance2 = {
    1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9};

constexpr std::array<double, 36> odom_twist_covariance = {
    1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};

constexpr std::array<double, 36> odom_twist_covariance2 = {
    1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9};
} // namespace ekf_config
} // namespace robot