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
