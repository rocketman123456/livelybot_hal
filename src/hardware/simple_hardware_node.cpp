#include "hardware/simple_hardware_node.h"

#include "humanoid/ankle_fk.h"

void SimpleHardwareNode::initialize()
{
    joint_posvel_pub = node.advertise<std_msgs::Float32MultiArray>("/joint_posvel", 10);
    joint_pub        = node.advertise<sensor_msgs::JointState>("/joint_states", 10);
    joint_sub        = node.subscribe("/joints_cmd", 1000, &SimpleHardwareNode::handleInput, this);

    // prepare data
    joint_posvel.data.resize(40);

    joint_cmd.position.resize(20);
    joint_cmd.velocity.resize(20);
    joint_cmd.effort.resize(20);
    joint_cmd.name.resize(20);

    joint_cmd.name[0]  = "r_hip_yaw_joint";
    joint_cmd.name[1]  = "r_hip_roll_joint";
    joint_cmd.name[2]  = "r_thigh_joint";
    joint_cmd.name[3]  = "r_calf_joint";
    joint_cmd.name[4]  = "r_ankle_pitch_joint";
    joint_cmd.name[5]  = "r_ankle_roll_joint";
    joint_cmd.name[6]  = "l_hip_yaw_joint";
    joint_cmd.name[7]  = "l_hip_roll_joint";
    joint_cmd.name[8]  = "l_thigh_joint";
    joint_cmd.name[9]  = "l_calf_joint";
    joint_cmd.name[10] = "l_ankle_pitch_joint";
    joint_cmd.name[11] = "l_ankle_roll_joint";
    joint_cmd.name[12] = "r_shoulder_pitch_joint";
    joint_cmd.name[13] = "r_shoulder_roll_joint";
    joint_cmd.name[14] = "r_shoulder_yaw_joint";
    joint_cmd.name[15] = "r_elbow_joint";
    joint_cmd.name[16] = "l_shoulder_pitch_joint";
    joint_cmd.name[17] = "l_shoulder_roll_joint";
    joint_cmd.name[18] = "l_shoulder_yaw_joint";
    joint_cmd.name[19] = "l_elbow_joint";

    joint_state.position.resize(20);
    joint_state.velocity.resize(20);
    joint_state.effort.resize(20);
    joint_state.name.resize(20);

    joint_state.name[0]  = "r_hip_yaw_joint";
    joint_state.name[1]  = "r_hip_roll_joint";
    joint_state.name[2]  = "r_thigh_joint";
    joint_state.name[3]  = "r_calf_joint";
    joint_state.name[4]  = "r_ankle_pitch_joint";
    joint_state.name[5]  = "r_ankle_roll_joint";
    joint_state.name[6]  = "l_hip_yaw_joint";
    joint_state.name[7]  = "l_hip_roll_joint";
    joint_state.name[8]  = "l_thigh_joint";
    joint_state.name[9]  = "l_calf_joint";
    joint_state.name[10] = "l_ankle_pitch_joint";
    joint_state.name[11] = "l_ankle_roll_joint";
    joint_state.name[12] = "r_shoulder_pitch_joint";
    joint_state.name[13] = "r_shoulder_roll_joint";
    joint_state.name[14] = "r_shoulder_yaw_joint";
    joint_state.name[15] = "r_elbow_joint";
    joint_state.name[16] = "l_shoulder_pitch_joint";
    joint_state.name[17] = "l_shoulder_roll_joint";
    joint_state.name[18] = "l_shoulder_yaw_joint";
    joint_state.name[19] = "l_elbow_joint";
}

void SimpleHardwareNode::processStep()
{
    // timeout handle
    mutex.write_lock();
    curr_time += dt;
    if (curr_time > timout)
    {
        for (double& torque : joint_cmd.effort)
        {
            torque = 0;
        }
    }
    mutex.write_unlock();

    // normal handle
    mutex.read_lock();
    for (int i = 0; i < 20; ++i)
    {
        driver->getMotor(i).setMixedControlInRad(0, 0, joint_cmd.effort[i], 0, 0);
    }
    mutex.read_unlock();

    // update motor data
    driver->update();

    // special handle to ankle joint
    float pos1;
    float pos2;
    pos1 = driver->getMotor(3).data.pos - driver->getMotor(2).data.pos;
    pos2 = driver->getMotor(9).data.pos - driver->getMotor(8).data.pos;

    double d  = 69.8 / 2.0;
    double L1 = 25.0;
    double h1 = 112.0;
    double h2 = 65.0;

    auto  ankle_l   = ankle_fk(d, L1, h1, h2, driver->getMotor(4).data.pos, driver->getMotor(5).data.pos);
    float ankle_l_x = ankle_l[0];
    float ankle_l_y = ankle_l[1];

    auto  ankle_r   = ankle_fk(d, L1, h2, h1, driver->getMotor(11).data.pos, driver->getMotor(10).data.pos);
    float ankle_r_x = ankle_r[0];
    float ankle_r_y = ankle_r[1];

    // prepate data
    prepareJointMsg(joint_posvel, driver);
    joint_posvel.data[3] = pos1;
    joint_posvel.data[9] = pos2;
    joint_posvel.data[4] = ankle_l_x;
    joint_posvel.data[5] = ankle_l_y;
    joint_posvel.data[11] = ankle_r_x;
    joint_posvel.data[10] = ankle_r_y;

    prepareJointMsg(joint_state, driver);
    joint_state.position[3]  = pos1;
    joint_state.position[9]  = pos2;
    joint_state.header.stamp = ros::Time::now();

    joint_posvel_pub.publish(joint_posvel);
    joint_pub.publish(joint_state);
}

void SimpleHardwareNode::handleInput(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    mutex.write_lock();
    curr_time = 0;
    joint_cmd.effort.resize(msg->data.size());
    for (int i = 0; i < msg->data.size(); ++i)
    {
        joint_cmd.effort[i] = msg->data[i];
    }
    mutex.write_unlock();
}

void SimpleHardwareNode::prepareJointMsg(sensor_msgs::JointState& msg, std::shared_ptr<HumanoidDriver> driver)
{
    for (int i = 0; i < 10; ++i)
    {
        auto& motor     = driver->getMotor(i);
        msg.position[i] = motor.getPositionRad();
        msg.velocity[i] = motor.getVelocityRad();
        msg.effort[i]   = motor.getTorque();
    }
}

void SimpleHardwareNode::prepareJointMsg(std_msgs::Float32MultiArray& msg, std::shared_ptr<HumanoidDriver> driver)
{
    for (int i = 0; i < 10; ++i)
    {
        auto& motor      = driver->getMotor(i);
        msg.data[i]      = motor.getPositionRad();
        msg.data[i + 10] = motor.getVelocityRad();
    }
}
