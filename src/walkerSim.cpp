/*
 *  @file    walkerSim.cpp
 *  @author  Gautam Balachandran
 *  @copyright MIT License
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @brief  Class file for the walkerSim Class
 *
 *  @section DESCRIPTION
 *
 *  This module is part of the ROS turtlebot walker tutorials.
 *  It defines the implementation of the walkerSim Class.
 *
 */

#include <iostream>
#include "../include/walkerSim.hpp"

walkerSim::walkerSim() {
    ROS_INFO_STREAM("Turtlebot Walker node initialized");
    // Initialize class params
    collision = false;
    // advertise the publisher topic with rosmaster
    pub = n.advertise <geometry_msgs::Twist>
                     ("/cmd_vel_mux/input/navi", 1000);
    // SUbscribe to the laserscan topic
    sub = n.subscribe("/scan", 500,
                      &walkerSim::laserScanCallback, this);
    // define the initial velocities
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    // publish the initial velocities
    pub.publish(msg);
}


walkerSim::~walkerSim() {
    // stop the turtlebot before exiting
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    // publish the  final velocities
    pub.publish(msg);
}


void walkerSim::laserScanCallback(
             const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Loop though the laserscan mesages to check collision
    for (int i = 0; i < msg->ranges.size(); ++i) {
    // Minimum threshold  distance for collision = 0.60
        if (msg->ranges[i] < 0.60) {
            collision = true;
            return;
        }
    }
    collision = false;
    return;
}

bool walkerSim::detectObstacle() {
    // return the collision flag
    return collision;
}

void walkerSim::navigateBot() {
    // Initialize the publisher freq
    ros::Rate loop_rate(10);
    // Implement till ros is running good
    while (ros::ok()) {
        // If obstacle is detected, turn the turtlebot
        if (detectObstacle()) {
            ROS_WARN_STREAM("Obstacle Detected! Turning TurtleBot!");
            // Stop the forward motion
            msg.linear.x = 0.0;
            // Rotate the bot
            msg.angular.z = -1.0;
        } else {
            ROS_INFO_STREAM("Path clear ahead! Moving Forward!");
            // If no obstacle, keep moving forward
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }
        // Publish the updated velocities
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
