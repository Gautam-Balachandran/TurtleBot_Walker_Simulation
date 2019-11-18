/*
 *  @file    walkerSim.hpp
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
 *  @brief  Header file for the walkerSim Class
 *
 *  @section DESCRIPTION
 *
 *  This module is part of the ROS turtlebot walker tutorials.
 *  It defines the header file for the program.
 *
 */

#ifndef INCLUDE_WALKERSIM_HPP_
#define INCLUDE_WALKERSIM_HPP_

#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class walkerSim {
 private:
    // ROS Node handle object
    ros::NodeHandle n;
    // ROS subscriber object
    ros::Subscriber sub;
    // ROS publisher object
    ros::Publisher pub;
    // Boolean flag to detect collision
    bool collision;
    // Publishes linear and angular velocities to the walker
    geometry_msgs::Twist msg;


 public:
   /**
    *   @brief  Constructor for walkerSim class
    *   @param  none
    *   @return void
    */
    walkerSim();
   /**
    *   @brief  Destructor for walkerSim class
    *   @param  none
    *   @return void
    */

    ~walkerSim();
   /**
    *   @brief  Callback function for subscriber to process laserScan data
    *            
    *   @param  pointer to LaserScan mesage 
    *
    *   @return void
    */
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

   /**
    *   @brief  Function to detect obstable nearby    
    *   @param  none
    *   @return true if object nearby, false otherwise
    */

    bool detectObstacle();

   /**
    *   @brief function to move the bot around 
    *   @param  none
    *   @return void
    */
    void navigateBot();
};

#endif    // INCLUDE_WALKERSIM_HPP_
