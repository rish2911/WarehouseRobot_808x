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