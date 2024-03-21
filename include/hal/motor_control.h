#pragma once

#include "hardware/motor.h"

// single motor control
class MotorControl
{
public:
    MotorControl()  = default;
    ~MotorControl() = default;

    void initialize(const motor_init_info_t& info);

    void setPositionDeg(std::shared_ptr<Motor> driver, double position);
    void setVelocityDeg(std::shared_ptr<Motor> driver, double velocity);
    void setPositionRad(std::shared_ptr<Motor> driver, double position);
    void setVelocityRad(std::shared_ptr<Motor> driver, double velocity);
    void setTorque(std::shared_ptr<Motor> driver, double torque);

    double getPositionRad() const;
    double getVelocityRad() const;
    double getPositionDeg() const;
    double getVelocityDeg() const;
    double getTorque() const;

private:
    uint16_t m_id            = 0;
    double   m_direction     = 1.0;
    double   m_offset        = 0.0;
    double   m_pos_scalar    = 0;
    double   m_vel_scalar    = 0;
    double   m_torque_scalar = 0;

    double m_position = 0.0;
    double m_velocity = 0.0;
    double m_torque   = 0.0;
};
