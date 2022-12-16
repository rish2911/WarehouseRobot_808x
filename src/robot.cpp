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
 * @brief Class declaration for Robot
 * @version 0.1
 * @date 2022-12-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include<../include/robot.hpp>
#include<XmlRpcValue.h>
#include<vector>


acme::Robot::Robot() {
    
    velocity_publisher = robot_nh_.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 100);
    fiducial_subscriber = robot_nh_.subscribe("/fiducial_transforms", 1, &acme::Robot::aruco_callback, this);
}
 
std::vector<std::vector<double>> acme::Robot::get_goals() {
    std::vector<double> explorer_goal_1{};
    std::vector<double> explorer_goal_2{};
    std::vector<double> explorer_goal_3{};
    std::vector<double> explorer_goal_4{};


    robot_nh_.getParam("/aruco_lookup_locations/target_1", explorer_goal_1);
    robot_nh_.getParam("/aruco_lookup_locations/target_2", explorer_goal_2);
    robot_nh_.getParam("/aruco_lookup_locations/target_3", explorer_goal_3);
    robot_nh_.getParam("/aruco_lookup_locations/target_4", explorer_goal_4);
    ROS_INFO("GOT GOALS 1");

    obj_locations_.push_back(explorer_goal_1);
    ROS_INFO("GOT GOALS 2");
    obj_locations_.push_back(explorer_goal_2);
    ROS_INFO("GOT GOALS 3");
    obj_locations_.push_back(explorer_goal_3);
    ROS_INFO("GOT GOALS 4");
    obj_locations_.push_back(explorer_goal_4);
    ROS_INFO("GOT GOALS 5");
    obj_locations_.push_back({-4.0, 2.5});
    ROS_INFO("GOT GOALS 6");

    return obj_locations_;
}

void acme::Robot::aruco_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    ROS_INFO("LOOKING FOR FIDUCIAL");

    // Start broadcasting if fiducial was found and robot has reached goal
    if(!msg->transforms.empty()){
        ROS_INFO("FOUND FIDUCIAL");
        // Create ROS TF broadcasting object
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        // Create the message for new frame at ArUco marker location
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        transformStamped.child_frame_id = "marker_frame";
        transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
        transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;
        transformStamped.transform.rotation.w = 1;
        ROS_INFO_STREAM("distance : "<<transformStamped.transform.translation.z);
        if(transformStamped.transform.translation.z < 2.0){

            // Record the ID of fiducial
            // this->current_fiducial_id = msg->transforms[0].fiducial_id;

            // Broadcast the ArUco frame to ROS TF
            br.sendTransform(transformStamped);
            ROS_INFO("PUBLISHED TRANSFORM"); 
            ROS_INFO("TURN OFF FIDUCIAL DETECTION");

            // Turn off Fiducial detection
            this->found_fiducial = true;
        }
    }
}

std::vector<double> acme::Robot::listener(tf2_ros::Buffer& tfBuffer){
    geometry_msgs::TransformStamped transformStamped;
    std::vector<double> final_pos;
    try {
        // Compute the transform between fiducial marker frame to map frame
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0), ros::Duration(4.0));
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        auto trans_z = transformStamped.transform.translation.z;

        // Compute a valid location between explorer and aruco location for which planner can compute a path
        ROS_INFO_STREAM(trans_x<<" "<<trans_y<<" "<<trans_z);
        // Map the ArUco location to ID
        final_pos.push_back(trans_x);
        final_pos.push_back(trans_y);
        ros::Duration(2.0).sleep();
        return final_pos;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void acme::Robot::move_to_goal(std::vector<double> goal) {
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = goal[0];//
    explorer_goal.target_pose.pose.position.y = goal[1];//
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    robot_client_.sendGoal(explorer_goal);
    // robot_client_.waitForResult();
    // while(robot_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    //     ROS_INFO("EXPLORER FOLLOWING GOAL");
    // }
    curr_pos_[0] = goal[0];
    curr_pos_[1] = goal[1];
    ROS_INFO("POSITION UPDATED");
    this->set_fiducial(false);
    return;

}

void acme::Robot::move_to_obj(std::vector<double> goal) {
    MoveBaseClient robot_client("/explorer/move_base", true);
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = curr_pos_[0]+threshold_dist*(goal[0] - curr_pos_[0]);//
    explorer_goal.target_pose.pose.position.y = curr_pos_[1]+threshold_dist*(goal[1]-curr_pos_[1]);//
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    robot_client_.sendGoal(explorer_goal);
    robot_client_.waitForResult();
    while(robot_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("ROBOT FOLLOWING GOAL");
    }

}

void acme::Robot::rotate() {
    ROS_INFO("ROBOT ROTATING");    
    // Create message for rotating explorer
    robot_msgs_.linear.x = 0;
    robot_msgs_.angular.z = 0.3;

    // Publish velocity message
    this->velocity_publisher.publish(robot_msgs_);
}

void acme::Robot::stop() {
    robot_msgs_.linear.x = 0;
    robot_msgs_.angular.z = 0;

    // Publish velocity message
    this->velocity_publisher.publish(robot_msgs_);
}

bool acme::Robot::check_fiducial() {
    return this->found_fiducial;
}

void acme::Robot::set_fiducial(bool value) {
    this->found_fiducial = value;
}

void acme::Robot::run(std::vector<std::vector<double>> goals) {
    std::vector<double> curr_goal;
    ros::Rate loop_rate(10);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while (!robot_client_.waitForServer(ros::Duration(5.0))) {
                ROS_INFO_STREAM(\
            "Waiting for the move_base action server to come up for ");
    }
    for (int i=0;i<goals.size();i++) {
        curr_goal = goals[i];
        this->move_to_goal(curr_goal);
        ROS_INFO("GOAL REACHED");
        if (i<4) {
            while(ros::ok()) {
                if (this->check_fiducial()) {
                    this->set_fiducial(false);
                    ROS_INFO("OBJECT FOUND");
                    break;
                }
                this->rotate();
                ros::spinOnce();
                loop_rate.sleep();
            }
            this->stop();
            ROS_INFO("APPROACHING OBJECT");
            curr_goal = this->listener(tfBuffer);
            this->move_to_obj(curr_goal);
            std::cout<<i;
            ROS_INFO("OBJECT COLLECTED");
        }
    }
}