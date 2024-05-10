#pragma once

#include "hardware/motor_control.h"

#include <hardware/robot.h>

#include <Eigen/Eigen>

class HumanoidDriver
{
public:
    HumanoidDriver()  = default;
    ~HumanoidDriver() = default;

    void initializeDriver();
    void initialize(const std::vector<motor_init_info_t>& motor_infos);

    void enableMotors();
    void disableMotors();
    void setMotorsZero();
    void update();

    MotorControl& getMotor(int index) { return motors[index]; }

    std::shared_ptr<RobotDriver> driver;
    std::vector<MotorControl>    motors;
};