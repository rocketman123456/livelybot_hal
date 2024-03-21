#include "hal/motor_control.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_motion");
    ros::NodeHandle node;
    ros::Rate rate(400);

    while (ros::ok())
    {
        // TODO

        rate.sleep();
    }

    return 0;
}