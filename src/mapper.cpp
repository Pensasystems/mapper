// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

// Standard includes
#include <mapper/mapper_class.h>
#include <ros/ros.h>
#include <signal.h>

mapper::MapperClass octomapper;

void SigInt(int sig) {  // Detect that ctrl+c has been pressed
    ROS_DEBUG("[mapper]: SIGINT detected! Terminating node!");
    octomapper.TerminateNode();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ROS_INFO("[mapper]: Starting...");

    ros::init(argc, argv, "mapper_node");
    ros::NodeHandle node("~");

    octomapper.Initialize(&node);

    // Shutdown ROS if sigint is detected
    signal(SIGINT, SigInt);

    ros::spin();

    return 0;
}
