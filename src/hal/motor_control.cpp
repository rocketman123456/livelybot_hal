#include "hal/motor_control.h"

// #define M_PI 3.14159265358979323846 /* pi */
#include <math.h>

void MotorControl::initialize(const motor_init_info_t& info)
{
    // m_can           = info.can;
    // m_id            = info.id;
    // m_direction     = info.direction;
    // m_offset        = info.offset;
    // m_pos_scalar    = info.pos_scalar;
    // m_vel_scalar    = info.vel_scalar;
    // m_torque_scalar = info.torque_scalar;
}

void MotorControl::setPositionDeg(std::shared_ptr<Motor> driver, double position)
{
    // double  result  = position * m_direction * m_pos_scalar * M_PI / 180.0;
    // uint8_t temp_id = m_can | (m_id + 1);
    // driver->set_motor_position(temp_id, result);
}

void MotorControl::setVelocityDeg(std::shared_ptr<Motor> driver, double velocity)
{
    // double  result  = velocity * m_direction * m_vel_scalar * M_PI / 180.0;
    // uint8_t temp_id = m_can | (m_id + 1);
    // driver->set_motor_velocity(temp_id, result);
}

void MotorControl::setPositionRad(std::shared_ptr<Motor> driver, double position)
{
    // double  result  = position * m_direction * m_pos_scalar;
    // uint8_t temp_id = m_can | (m_id + 1);
    // driver->set_motor_position(temp_id, result);
}

void MotorControl::setVelocityRad(std::shared_ptr<Motor> driver, double velocity)
{
    // double  result  = velocity * m_direction * m_vel_scalar;
    // uint8_t temp_id = m_can | (m_id + 1);
    // driver->set_motor_velocity(temp_id, result);
}

void MotorControl::setTorque(std::shared_ptr<Motor> driver, double torque)
{
    // double  result  = torque * m_direction * m_torque_scalar;
    // uint8_t temp_id = m_can | (m_id + 1);
    // driver->set_motor_torque(temp_id, result);
}

// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------

double MotorControl::getPositionRad() const { return m_position; }

double MotorControl::getVelocityRad() const { return m_velocity; }

double MotorControl::getPositionDeg() const { return m_position * 180.0 / M_PI; }

double MotorControl::getVelocityDeg() const { return m_velocity * 180.0 / M_PI; }

double MotorControl::getTorque() const { return m_torque; }