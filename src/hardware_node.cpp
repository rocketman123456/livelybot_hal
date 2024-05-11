#include "hardware/humanoid_driver.h"
#include "hardware/robot.h"
#include "hardware/simple_hardware_node.h"

#include <ros/ros.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <unistd.h> // usleep
#include <vector>

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] %v");
    spdlog::set_level(spdlog::level::debug);

    // init ros
    ros::init(argc, argv, "hardware_node", ros::init_options::AnonymousName);
    ros::NodeHandle node;
    ros::Rate       rate(500);

    std::shared_ptr<HumanoidDriver> driver = std::make_shared<HumanoidDriver>();
    std::vector<motor_init_info_t>  motor_infos;

    // TODO : read from config file
    spdlog::debug("init robot driver");
    driver->initializeDriver();

    // TODO : read from config file
    spdlog::debug("init motors");
    motor_infos.emplace_back(15, -1.0, 0);  // 0
    motor_infos.emplace_back(14, 1.0, 0);   // 1
    motor_infos.emplace_back(13, 1.0, 0);   // 2
    motor_infos.emplace_back(12, -1.0, 0);  // 3
    motor_infos.emplace_back(11, -1.0, 0);  // 4
    motor_infos.emplace_back(10, 1.0, 0);   // 5
    motor_infos.emplace_back(5, -1.0, 0);   // 6
    motor_infos.emplace_back(4, 1.0, 0);    // 7
    motor_infos.emplace_back(3, -1.0, 0);   // 8
    motor_infos.emplace_back(2, 1.0, 0);    // 9
    motor_infos.emplace_back(1, 1.0, 0);    // 10
    motor_infos.emplace_back(0, -1.0, 0);   // 11
    motor_infos.emplace_back(16, -1.0, 0);  // 12
    motor_infos.emplace_back(17, 1.0, 0);   // 13
    motor_infos.emplace_back(18, -1.0, 0);  // 14
    motor_infos.emplace_back(19, -1.0, 0);  // 15
    motor_infos.emplace_back(6, 1.0, 0);    // 16
    motor_infos.emplace_back(7, 1.0, 0);    // 17
    motor_infos.emplace_back(8, -1.0, 0);   // 18
    motor_infos.emplace_back(9, 1.0, 0);    // 19
    driver->initialize(motor_infos);

    // first time init, to make sure motor is ready
    spdlog::debug("enable motors");
    for (int i = 0; i < driver->motors.size(); ++i)
    {
        driver->motors[i].setMixedControlInRad(0, 0, 0, 0, 0);
    }
    driver->update();

    ros::AsyncSpinner sub_spinner(2);
    sub_spinner.start();

    // create publisher and subscriber
    SimpleHardwareNode handle(node, driver);
    handle.initialize();

    spdlog::debug("start control loop.");
    while (ros::ok())
    {
        handle.processStep();

        rate.sleep();
        ros::spinOnce();
    }

    spdlog::debug("disable motors");
    for (int i = 0; i < driver->motors.size(); ++i)
    {
        driver->motors[i].setMixedControlInRad(0, 0, 0, 0, 0);
    }
    driver->update();

    for (auto& thread : driver->driver->m_receive_threads)
    {
        thread.join();
    }

    ros::waitForShutdown();
    return 0;
}
