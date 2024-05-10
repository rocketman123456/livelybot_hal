#pragma once

#include "hardware/motor_data.h"

#include <cstdint>

struct motor_init_info_t
{
    motor_init_info_t() = default;

    motor_init_info_t(uint16_t id_, double dir, double off)
        : id(id_)
        , direction(dir)
        , offset(off)
    {}

    uint16_t id         = 0;
    double   direction  = 1.0;
    double   offset     = 0.0;
    double   pos_scalar = 1.0;
    double   vel_scalar = 1.0;
    double   tau_scalar = 1.0;
};

// common control procedure:
//      enable -> set control parameter -> update
class MotorControl
{
public:
    MotorControl()  = default;
    ~MotorControl() = default;

    void initialize(const motor_init_info_t& info);

    // TODO : add limit for safety
    void setPosLimit(double lower, double upper);
    void setVelLimit(double lower, double upper);
    void setTauLimit(double lower, double upper);

    void setMixedControlInDeg(double pos, double vel, double tau, double kp, double kd);
    void setMixedControlInRad(double pos, double vel, double tau, double kp, double kd);

    double getPositionRad() const;
    double getVelocityRad() const;
    double getPositionDeg() const;
    double getVelocityDeg() const;
    double getTorque() const;

    // update data with infos
    void update();

    uint16_t id         = 0;
    double   direction  = 1.0;
    double   offset     = 0.0;
    double   pos_scalar = 1.0;
    double   vel_scalar = 1.0;
    double   tau_scalar = 1.0;

    double torque_upper_limit = 12.0;
    double torque_lower_limit = -12.0;

    motor_data_t data;
};
