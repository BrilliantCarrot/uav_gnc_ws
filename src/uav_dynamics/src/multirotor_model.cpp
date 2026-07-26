#include "uav_dynamics/multirotor_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

double clamp(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

bool solve4x4(std::array<std::array<double, 4>, 4> a,
              std::array<double, 4> b,
              std::array<double, 4>& x)
{
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        double max_abs = std::abs(a[col][col]);
        for (int row = col + 1; row < 4; ++row) {
            const double v = std::abs(a[row][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = row;
            }
        }

        if (max_abs < 1e-9) {
            return false;
        }

        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }

        const double diag = a[col][col];
        for (int j = col; j < 4; ++j) {
            a[col][j] /= diag;
        }
        b[col] /= diag;

        for (int row = 0; row < 4; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            for (int j = col; j < 4; ++j) {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }

    x = b;
    return true;
}

}  // namespace

bool MultirotorModel::configure(const MultirotorConfig& config)
{
    if (config.rotors.empty()) {
        configured_ = false;
        return false;
    }

    config_ = config;
    rotor_thrusts_.assign(config_.rotors.size(), 0.0);
    rotor_omegas_.assign(config_.rotors.size(), 0.0);
    debug_.thrust_cmd.assign(config_.rotors.size(), 0.0);
    debug_.thrust_actual.assign(config_.rotors.size(), 0.0);
    debug_.omega_cmd.assign(config_.rotors.size(), 0.0);
    debug_.omega_actual.assign(config_.rotors.size(), 0.0);
    configured_ = true;
    return true;
}

void MultirotorModel::reset()
{
    std::fill(rotor_thrusts_.begin(), rotor_thrusts_.end(), 0.0);
    std::fill(rotor_omegas_.begin(), rotor_omegas_.end(), 0.0);
    debug_.thrust_cmd = rotor_thrusts_;
    debug_.thrust_actual = rotor_thrusts_;
    debug_.omega_cmd = rotor_omegas_;
    debug_.omega_actual = rotor_omegas_;
}

Input MultirotorModel::update(const Input& desired_wrench, double dt, double altitude_m)
{
    if (!configured_) {
        return desired_wrench;
    }

    const std::vector<double> thrust_cmd = allocateRotorThrusts(desired_wrench);
    debug_.thrust_cmd = thrust_cmd;

    const double tau = config_.motor_tau;
    const double alpha = (tau > 1e-6) ? clamp(dt / tau, 0.0, 1.0) : 1.0;

    if (config_.use_rpm_propeller) {
        const double kT = std::max(config_.prop_thrust_coeff, 1e-12);
        const double omega_max = effectiveOmegaMax();

        for (std::size_t i = 0; i < rotor_thrusts_.size(); ++i) {
            const double clipped_thrust_cmd = clamp(
                thrust_cmd[i], config_.rotors[i].thrust_min, config_.rotors[i].thrust_max);
            const double omega_cmd = clamp(std::sqrt(clipped_thrust_cmd / kT), 0.0, omega_max);
            rotor_omegas_[i] += alpha * (omega_cmd - rotor_omegas_[i]);
            rotor_omegas_[i] = clamp(rotor_omegas_[i], 0.0, omega_max);
            rotor_thrusts_[i] = clamp(kT * rotor_omegas_[i] * rotor_omegas_[i],
                                      config_.rotors[i].thrust_min,
                                      config_.rotors[i].thrust_max);
            debug_.omega_cmd[i] = omega_cmd;
        }
    } else {
        for (std::size_t i = 0; i < rotor_thrusts_.size(); ++i) {
            rotor_thrusts_[i] += alpha * (thrust_cmd[i] - rotor_thrusts_[i]);
            debug_.omega_cmd[i] = 0.0;
            rotor_omegas_[i] = 0.0;
        }
    }

    debug_.thrust_actual = rotor_thrusts_;
    debug_.omega_actual = rotor_omegas_;
    return rotorThrustsToWrench(rotor_thrusts_, altitude_m);
}

std::vector<double> MultirotorModel::allocateRotorThrusts(const Input& desired_wrench) const
{
    const std::size_t n = config_.rotors.size();
    std::vector<std::array<double, 4>> b_cols(n);

    for (std::size_t i = 0; i < n; ++i) {
        const RotorConfig& r = config_.rotors[i];
        b_cols[i] = {
            1.0,
            r.position_body.y,
            -r.position_body.x,
            r.yaw_torque_sign * config_.yaw_torque_to_thrust
        };
    }

    std::array<double, 4> cmd = {
        std::max(0.0, desired_wrench.thrust_body.z),
        desired_wrench.moment_body.x,
        desired_wrench.moment_body.y,
        desired_wrench.moment_body.z
    };

    std::array<std::array<double, 4>, 4> bbt{};
    for (const auto& col : b_cols) {
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                bbt[r][c] += col[r] * col[c];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        bbt[i][i] += 1e-8;
    }

    std::array<double, 4> lambda{};
    std::vector<double> thrusts(n, cmd[0] / static_cast<double>(n));
    if (solve4x4(bbt, cmd, lambda)) {
        for (std::size_t i = 0; i < n; ++i) {
            double thrust = 0.0;
            for (int row = 0; row < 4; ++row) {
                thrust += b_cols[i][row] * lambda[row];
            }
            thrusts[i] = thrust;
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        const RotorConfig& r = config_.rotors[i];
        thrusts[i] = clamp(thrusts[i], r.thrust_min, r.thrust_max);
    }

    return thrusts;
}

Input MultirotorModel::rotorThrustsToWrench(
    const std::vector<double>& rotor_thrusts, double altitude_m) const
{
    Input out;
    out.thrust_body = {0.0, 0.0, 0.0};
    out.moment_body = {0.0, 0.0, 0.0};

    const double ge = groundEffectFactor(altitude_m);

    for (std::size_t i = 0; i < rotor_thrusts.size(); ++i) {
        const RotorConfig& r = config_.rotors[i];
        const double thrust = rotor_thrusts[i] * ge;
        const Vec3 force_body{0.0, 0.0, thrust};
        const Vec3 arm_moment = Vec3::cross(r.position_body, force_body);
        const double yaw_torque = config_.use_rpm_propeller
            ? r.yaw_torque_sign * config_.prop_torque_coeff * rotor_omegas_[i] * rotor_omegas_[i] * ge
            : r.yaw_torque_sign * config_.yaw_torque_to_thrust * thrust;
        const Vec3 yaw_moment{0.0, 0.0, yaw_torque};

        out.thrust_body += force_body;
        out.moment_body += arm_moment + yaw_moment;
    }

    return out;
}

double MultirotorModel::groundEffectFactor(double altitude_m) const
{
    if (!config_.enable_ground_effect || config_.ground_effect_altitude_m <= 1e-6) {
        return 1.0;
    }

    const double h = std::max(0.0, altitude_m);
    if (h >= config_.ground_effect_altitude_m) {
        return 1.0;
    }

    const double proximity = 1.0 - h / config_.ground_effect_altitude_m;
    return 1.0 + std::max(0.0, config_.ground_effect_gain) * proximity * proximity;
}

double MultirotorModel::effectiveOmegaMax() const
{
    const double nominal = std::max(config_.battery_voltage_nominal, 1e-6);
    const double voltage_scale = clamp(config_.battery_voltage / nominal, 0.0, 1.2);
    return std::max(0.0, config_.motor_omega_max * voltage_scale);
}
