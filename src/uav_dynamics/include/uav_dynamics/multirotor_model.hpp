#pragma once

#include <string>
#include <vector>

#include "uav_dynamics/sixdof.hpp"

struct RotorConfig {
    std::string name;
    Vec3 position_body;
    double yaw_torque_sign{1.0};
    double thrust_min{0.0};
    double thrust_max{15.0};
};

struct MultirotorConfig {
    std::string airframe_name{"generic_multirotor"};
    std::vector<RotorConfig> rotors;
    double motor_tau{0.04};
    double yaw_torque_to_thrust{0.016};
    bool use_rpm_propeller{false};
    double prop_thrust_coeff{1.8e-5};  // N/(rad/s)^2
    double prop_torque_coeff{2.8e-7};  // N*m/(rad/s)^2
    double motor_omega_max{900.0};     // rad/s
    double battery_voltage_nominal{14.8};
    double battery_voltage{14.8};
    bool enable_ground_effect{false};
    double ground_effect_altitude_m{0.8};
    double ground_effect_gain{0.12};
};

struct ActuatorDebug {
    std::vector<double> thrust_cmd;
    std::vector<double> thrust_actual;
    std::vector<double> omega_cmd;
    std::vector<double> omega_actual;
};

class MultirotorModel {
public:
    bool configure(const MultirotorConfig& config);
    void reset();

    Input update(const Input& desired_wrench, double dt, double altitude_m = 1e9);
    const ActuatorDebug& debug() const { return debug_; }
    const MultirotorConfig& config() const { return config_; }
    bool configured() const { return configured_; }

private:
    std::vector<double> allocateRotorThrusts(const Input& desired_wrench) const;
    Input rotorThrustsToWrench(const std::vector<double>& rotor_thrusts, double altitude_m) const;
    double groundEffectFactor(double altitude_m) const;
    double effectiveOmegaMax() const;

    MultirotorConfig config_;
    std::vector<double> rotor_thrusts_;
    std::vector<double> rotor_omegas_;
    ActuatorDebug debug_;
    bool configured_{false};
};
