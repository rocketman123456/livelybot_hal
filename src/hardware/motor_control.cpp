#include "hardware/motor_control.h"

#include <spdlog/spdlog.h>

void MotorControl::initialize(const motor_init_info_t& info)
{
    id         = info.id;
    direction  = info.direction;
    offset     = info.offset;
    pos_scalar = info.pos_scalar;
    vel_scalar = info.vel_scalar;
    tau_scalar = info.tau_scalar;
}

void MotorControl::setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd)
{
    double clamp_torque;
    if (tau > torque_upper_limit)
    {
        tau = torque_upper_limit;
    }
    else if (tau < torque_lower_limit)
    {
        tau = torque_lower_limit;
    }

    data.pos_des = pos * direction * pos_scalar * M_PI / 180.0;
    data.vel_des = vel * direction * vel_scalar * M_PI / 180.0;
    data.tau_des = tau * direction * tau_scalar;
    data.kp      = kp;
    data.kd      = kd;
}

void MotorControl::setMixedControlInRad(double pos, double vel, double tau, double kp, double kd)
{
    double clamp_torque;
    if (tau > torque_upper_limit)
    {
        tau = torque_upper_limit;
    }
    else if (tau < torque_lower_limit)
    {
        tau = torque_lower_limit;
    }

    data.pos_des = direction * (pos * pos_scalar) - offset;
    data.vel_des = direction * vel * vel_scalar;
    data.tau_des = direction * tau * tau_scalar;
    data.kp      = kp;
    data.kd      = kd;
}

void MotorControl::update()
{
    // handle data
    // data.pos /= pos_scalar;
    data.pos += offset;
    data.pos *= direction;
    data.vel *= direction;
    data.tau *= direction;
}

// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------

double MotorControl::getPositionRad() const { return data.pos; }
double MotorControl::getVelocityRad() const { return data.vel; }
double MotorControl::getPositionDeg() const { return data.pos * 180.0 / M_PI; }
double MotorControl::getVelocityDeg() const { return data.vel * 180.0 / M_PI; }
double MotorControl::getTorque() const { return data.tau; }
