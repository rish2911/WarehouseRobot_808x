
/******************************************************************************
 * MIT License
 * 
 * Copyright (c) 2022 Adithya Singh, Rishabh Singh, Divyansh Agrawal
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 ******************************************************************************/

/**
 * @file robot.hpp
 * @author Divyansh Agrawal
 * @author Rishabh Singh 
 * @author Adithya Singh
 * @brief Class declaration for Robot
 * @version 0.1
 * @date 2022-12-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_
#include <array>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <cstring>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/**
 * @brief 
 * 
 */
namespace acme {
    /**
     * @brief 
     * 
     */
    class Robot {
        public:
            /**
             * @brief Function for robot to move to goal
             * 
             * @param goal 
             */
            Robot();
            void move_to_goal(std::vector<double> goal);
            /**
             * @brief Function for robot to move to object
             * 
             * @param obj_loc 
             */
            void move_to_obj(std::vector<double> obj_loc);
            /**
             * @brief Function to retrieve goals from ROS Parameter
             * 
             */
            std::vector<std::vector<double>> get_goals();
            /**
             * @brief Aruco call back
             * 
             */
            void aruco_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
            /**
             * @brief Function to retrieve frame transform
             * 
             * @return std::vector<double> 
             */
            std::vector<double> listener(tf2_ros::Buffer&);
            /**
             * @brief Function to run robot
             * 
             */
            void run(std::vector<std::vector<double>>);
            /**
             * @brief Function to rotate robot
             * 
             */
            void rotate();
            /**
             * @brief Function to stop robot
             * 
             */

            void stop();
            /**
             * @brief Set the fiducial object
             * 
             */
            void set_fiducial(bool);
            /**
             * @brief Get fiducial value
             * 
             * @return true 
             * @return false 
             */
            bool check_fiducial();

        private:
            /**
             * @brief ROS node handler
             * 
             */
            ros::NodeHandle robot_nh_;
            /**
             * @brief Container for all locatins
             * 
             */
            std::vector<std::vector<double>> obj_locations_;
            /**
             * @brief Function to store curr position of robot
             * 
             */
            std::vector<double> curr_pos_{-4.0, 2.5};
            /**
             * @brief Threshold for pickup distance
             * 
             */
            double threshold_dist=0.5;
            /**
             * @brief Robot goal message
             * 
             */
            move_base_msgs::MoveBaseGoal robot_goal_;
            /**
             * @brief ROS cmd_vel msg
             * 
             */
            geometry_msgs::Twist robot_msgs_;
            /**
             * @brief ROS publisher for velocity
             * 
             */
            ros::Publisher velocity_publisher;
            /**
             * @brief ROS Subscriber for fiducial
             * 
             */
            ros::Subscriber fiducial_subscriber;
            /**
             * @brief Check fiducial status
             * 
             */
            bool found_fiducial=false;
            /**
             * @brief ROS Action client
             * 
             */
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> robot_client_{"/explorer/move_base", true};
            
    };
}
#endif  // INCLUDE_ROBOT_HPP_