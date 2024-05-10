#include "hardware/humanoid_driver.h"

#include <iostream>
#include <unistd.h>

void HumanoidDriver::initializeDriver() { driver = std::make_shared<RobotDriver>(); }

void HumanoidDriver::initialize(const std::vector<motor_init_info_t>& infos)
{
    std::cout << "motor count: " << infos.size() << std::endl;
    motors.resize(infos.size());
    for (size_t i = 0; i < infos.size(); ++i)
    {
        motors[i].initialize(infos[i]);
    }
}

// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------

void HumanoidDriver::update()
{
    for (int i = 0; i < motors.size(); ++i)
    {
        auto& motor = motors[i];
        driver->m_motors[motor.id]->fresh_cmd(0.0, 0.0, motor.data.tau_des, 0.0, 0.0);
    }

    driver->motor_send();

    for (int i = 0; i < motors.size(); ++i)
    {
        auto& motor = motors[i];
        auto* state = driver->m_motors[motor.id]->get_current_motor_state();

        motors[i].data.pos = state->position;
        motors[i].data.vel = state->velocity;
        motors[i].data.tau = state->torque;
        motors[i].update();
    }
}
