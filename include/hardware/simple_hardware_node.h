#pragma once

#include "hardware/humanoid_driver.h"
#include "utils/fast_mutex.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

class SimpleHardwareNode
{
public:
    explicit SimpleHardwareNode(ros::NodeHandle& node, std::shared_ptr<HumanoidDriver> driver)
        : node(node)
        , driver(driver)
    {}

    ~SimpleHardwareNode() = default;

    void initialize();
    void processStep();
    void handleInput(const std_msgs::Float32MultiArray::ConstPtr& msg);

private:
    static void prepareJointMsg(std_msgs::Float32MultiArray& msg, std::shared_ptr<HumanoidDriver> driver);
    static void prepareJointMsg(sensor_msgs::JointState& msg, std::shared_ptr<HumanoidDriver> driver);

public:
    ros::NodeHandle&             node;
    std::shared_ptr<HumanoidDriver> driver;

    rw_fast_mutex mutex;

    ros::Publisher  joint_posvel_pub;
    ros::Publisher  joint_pub;
    ros::Subscriber joint_sub;

    std_msgs::Float32MultiArray joint_posvel;
    sensor_msgs::JointState     joint_state;
    sensor_msgs::JointState     joint_cmd;

    double curr_time = 0;
    double timout    = 1.0;
    double dt        = 1.0 / 500.0;
};
