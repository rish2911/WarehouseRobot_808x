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
 * @file robot.cpp
 * @author Divyansh Agrawal
 * @author Rishabh Singh 
 * @author Adithya Singh
 * @brief Methods for the Robot class
 * @version 0.1
 * @date 2022-12-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/robot.h"

std::array<std::array<double, 2>, 5>  acme::Robot::get_goal() {
    ros::NodeHandle robot_nh_;
    std::array<XmlRpc::XmlRpcValue, 4> pos_list;
    char target_id[] = {'1', '2', '3', '4'};
    std::string aruco_lookup_locations = "/aruco_lookup_locations/target_";

    for (int i = 0; i <4; i++) {
        robot_nh_.getParam(aruco_lookup_locations + target_id[i], pos_list[i]);
    }

    for (int i = 0; i < 4; i++) {
        ROS_ASSERT(pos_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    }

    for (int i = 0; i < 4; i ++) {
        for (int32_t j = 0; j < pos_list[i].size(); ++j) {
            ROS_ASSERT(pos_list[i][j].getType() == \
            XmlRpc::XmlRpcValue::TypeDouble);
            aruco_locations_.at(i).at(j) = static_cast<double>(pos_list[i][j]);
        }
    }
    aruco_locations_.at(4).at(0) = -4;  // home location for explorer
    aruco_locations_.at(4).at(1) = 2.5;
    return aruco_locations_;
}



void acme::Robot::listen(tf2_ros::Buffer& tfBuffer) {
  ros::NodeHandle robot_nh_;
  ros::Duration(1.0).sleep();
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", \
    "marker_frame", ros::Time(0));
    auto trans_x = (transformStamped.transform.translation.x + \
    m_goal.target_pose.pose.position.x)/2;
    auto trans_y = (transformStamped.transform.translation.y + \
    m_goal.target_pose.pose.position.y)/2;
    auto trans_z = transformStamped.transform.translation.z;
    ros::Duration(4.0).sleep();
    marker_loc.at(m_aruco_id).at(0) = trans_x;
    marker_loc.at(m_aruco_id).at(1) = trans_y;
    marker_loc.at(4).at(0) = -4;
    marker_loc.at(4).at(1) =  3.5;
    ROS_INFO_STREAM("Position of marker with ID " \
    << m_aruco_id << " in map frame: ["
      << trans_x << ", "
      << trans_y << ", "
      << trans_z << "]");
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
}
}
