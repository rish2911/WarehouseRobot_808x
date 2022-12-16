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
 * @file main.cpp
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

 #include "../include/robot.hpp"
 #include <vector>
 #include <ros/ros.h>

 int main(int argc, char** argv) {
    ROS_INFO("Starting node");
    ros::init(argc, argv, "collector_robot");

    std::vector<std::vector<double>> goals;
    acme::Robot robot;
    ROS_INFO("GETTING GOALS");
    goals = robot.get_goals();
    ROS_INFO("GOT GOALS");
    robot.run(goals);
 }