
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
#include <string>
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
/**
 * @brief
 *
 */
namespace acme {
    /**
     * @brief Class to represent a robot in an environment
     *
     */
    class Robot {
     public:
        /**
         * @brief Default Constructor for Robot object
         *
         */
        Robot() : robot_client_{"/explorer/move_base", true}, \
        robot_name_{"explorer"}
        {
        }
        /**
         * @brief Construct a new Robot object
         *
         * @param name: std::string
         */
        Robot(std::string name) : robot_name_{name}, \
        robot_client_{"/" + name + "/move_base", true} {
        }

        /**
         * @brief Allows the robots to move to goal location. In case of the explorer,
         * it also turns on spot and finds the aruco marker and broadcasts its
         * location wrt global coordinates after transformation.
         *
         * @param goal: std::array<std::array<double, 2>, 5>
         */
        void move_to_goal(std::array<std::array<double, 2>, 5> goal);
        /**
         * @brief
         *
         * @param obj_loc
         */
        void move_to_obj(std::vector<double> obj_loc);
        /**
         * @brief
         *
         */
        std::vector<std::vector<double>> get_goals();
        /**
         * @brief
         *
         */
        void acme::Robot::aruco_callback(const fiducial_msgs::\
        FiducialTransformArray::ConstPtr& m_msg);

        /**
         * @brief Listens to tf2_ros::Buffer and uses tf transform to get the coordinates of the markers in map frame.
         * Used to get goal locations for follower robot.
         * 
         * @param tfBuffer  
         */
        void listen(const tf2_ros::Buffer& tfBuffer);

     private:
        ros::NodeHandle robot_nh_;
        std::string robot_name_;  // robot name
        int32_t m_aruco_id{};  // Index for aruco locations
        std::vector<std::vector<double>> obj_locations_;
        //
        std::vector<double> final_obj_pos_;
        //
        double threshold_dist_;
        //
        move_base_msgs::MoveBaseGoal robot_goal_;
        //
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> \
        robot_client_;  // move base client
        geometry_msgs::Twist robot_msgs_;
        bool saw_marker{false};  // Flag for marker spotting
    };
}  // namespace acme
#endif  // INCLUDE_ROBOT_HPP_
