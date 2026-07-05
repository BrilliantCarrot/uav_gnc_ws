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
};

struct ActuatorDebug {
    std::vector<double> thrust_cmd;
    std::vector<double> thrust_actual;
};

class MultirotorModel {
public:
    bool configure(const MultirotorConfig& config);
    void reset();

    Input update(const Input& desired_wrench, double dt);
    const ActuatorDebug& debug() const { return debug_; }
    const MultirotorConfig& config() const { return config_; }
    bool configured() const { return configured_; }

private:
    std::vector<double> allocateRotorThrusts(const Input& desired_wrench) const;
    Input rotorThrustsToWrench(const std::vector<double>& rotor_thrusts) const;

    MultirotorConfig config_;
    std::vector<double> rotor_thrusts_;
    ActuatorDebug debug_;
    bool configured_{false};
};
