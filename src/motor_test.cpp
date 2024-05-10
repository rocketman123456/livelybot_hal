#include "hardware/humanoid_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdio>
#include <unistd.h> // usleep
#include <vector>

bool is_near(float a, float b, float tol = 1e-4) { return abs(abs(a) - abs(b)) < tol; }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware_node", ros::init_options::AnonymousName);
    ros::NodeHandle node;
    ros::Rate       rate(500);

    // create publisher
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 10);

    spdlog::set_pattern("[%^%l%$] %v");
    spdlog::set_level(spdlog::level::debug);

    std::shared_ptr<BipedDriver> driver = std::make_shared<BipedDriver>();

    std::vector<can_init_info_t>   can_infos;
    std::vector<motor_init_info_t> motor_infos;

    can_infos.emplace_back("can0");
    can_infos.emplace_back("can1");
    driver->initialize(can_infos);

    float delta = 360.0 / 7.75 / 180.0 * M_PI;
    printf("delta: %f\n", delta);

    motor_infos.emplace_back(0, 1, 1.0, delta);
    motor_infos.emplace_back(0, 2, -1.0, delta);
    motor_infos.emplace_back(0, 3, -1.0, delta);
    motor_infos.emplace_back(0, 4, 1.0, delta);
    motor_infos.emplace_back(0, 5, 1.0, delta);
    motor_infos.emplace_back(1, 6, 1.0, delta);
    motor_infos.emplace_back(1, 7, -1.0, delta);
    motor_infos.emplace_back(1, 8, 1.0, delta);
    motor_infos.emplace_back(1, 9, -1.0, delta);
    motor_infos.emplace_back(1, 10, -1.0, delta);
    driver->initialize(motor_infos);

    driver->enableMotors();
    driver->update();

    // create message
    sensor_msgs::JointState state;
    state.position.resize(10);
    state.velocity.resize(10);
    state.effort.resize(10);
    state.name.resize(10);

    state.name[0] = "LeftHipYaw_Joint";
    state.name[1] = "LeftHipRoll_Joint";
    state.name[2] = "LeftHipPitch_Joint";
    state.name[3] = "LeftKnee_Joint";
    state.name[4] = "LeftAnkle_Joint";
    state.name[5] = "RightHipYaw_Joint";
    state.name[6] = "RightHipRoll_Joint";
    state.name[7] = "RightHipPitch_Joint";
    state.name[8] = "RightKnee_Joint";
    state.name[9] = "RightAnkle_Joint";

    spdlog::debug("start modern robotics init");

    double dt    = 0.002;
    double angle = 0;
    double range = 0.05;
    double vel   = 15;

    double   kp     = 1.0;
    double   kp_max = 20.0;
    double   dkp    = 10.0 / 500.0;
    uint64_t count  = 0;

    spdlog::debug("Start control loop.");
    while (ros::ok())
    {
        // double pos_x = sin(angle) * range;
        // double pos_y = cos(angle) * range;
        // double pos_z = sin(angle) * range;
        // angle += dt * vel;

        // // clang-format off
        // Eigen::MatrixXd target(4, 4);
        // target <<
        //     1, 0, 0, 0,
        //     0, 1, 0, 0,
        //     0, 0, 1, -0.30 + pos_z,
        //     0, 0, 0, 1;
        // // clang-format on

        // mr::IKinSpace(Slist, M, target, q, 1.0, 0.001);

        // // handle ankle joint
        // {
        //     q(4) = q(3) + q(4);
        // }

        // if(kp < kp_max)
        // {
        //     kp += dkp;
        //     printf("kp: %f\n", kp);
        // }

        // auto& motor = driver->getMotor(0);
        // motor.setMixedControlInRad(0, 0, 0.1, 0, 00);

        // for (int i = 0; i < 5; ++i)
        // {
        //     auto& motor = driver->getMotor(i);
        //     motor.setMixedControlInRad(q(i), 0, 0, kp, 0.1);
        // }
        // for (int i = 5; i < 10; ++i)
        // {
        //     auto& motor = driver->getMotor(i);
        //     motor.setMixedControlInRad(q(i-5), 0, 0, kp, 0.1);
        // }
        driver->update();

        // read ankle angle handle
        {
            auto& motor     = driver->getMotor(4);
            motor.data.pos = motor.data.pos - driver->getMotor(3).data.pos;
        }
        {
            auto& motor     = driver->getMotor(9);
            motor.data.pos = motor.data.pos - driver->getMotor(8).data.pos;
        }

        // finish
        for (int i = 0; i < 10; ++i)
        {
            auto& motor = driver->getMotor(i);
            // std::cout << "motor " << i << " pos: " << motor._data.pos * 180.0 / M_PI << std::endl;
            state.position[i] = motor.data.pos;
        }

        // publish joint state
        state.header.stamp = ros::Time::now();
        joint_pub.publish(state);

        rate.sleep();
    }

    driver->disableMotors();
    driver->update();

    return 0;
}