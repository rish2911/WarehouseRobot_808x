/**
 * @file robot.cpp
 * @author Rishabh Singh (Driver)
 * @author Divyansh Agrawal (Design Keeper)
 * @author Adithya Singh (Navigator)
 * @brief This file contains the library and implementation for robot class
 * @version 0.1
 * @date 2022-12-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/robot.hpp"

void acme::Robot::aruco_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& m_msg)
{
  ros::NodeHandle m_nh;
 
if (!m_msg->transforms.empty())
 {  ROS_INFO_STREAM("Scan succesful. Found the marker with ID "<< m_msg->transforms[0].fiducial_id);
    m_aruco_id = m_msg->transforms[0].fiducial_id;
    static tf2_ros::TransformBroadcaster brc;
    geometry_msgs::TransformStamped transformStamped;
     
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    //creation of marker_frame with relative to camera frame at given distance
    //which is exactly what we are seeing in the camera frame
    transformStamped.transform.translation.x = m_msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = m_msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = m_msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = m_msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = m_msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = m_msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = m_msg->transforms[0].transform.rotation.w;
    

  if((transformStamped.transform.translation.x)*(transformStamped.transform.translation.x) + (transformStamped.transform.translation.y)*(transformStamped.transform.translation.y) < 4)
  {
    ROS_INFO_STREAM("Broadcasting marker "<<m_aruco_id<<" location");
    brc.sendTransform(transformStamped);
    saw_marker = true;
   
  }
  else
  {
    ROS_INFO("Ignored detected marker as it is too far.");
  }
 }
}

void acme::Robot::move_to_goal(std::array<std::array<double, 2>, 5> goal_loc)
{
 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 ros::NodeHandle robot_nh_; 

  bool goal_sent = false;
  int i = 0;
  tf2_ros::Buffer tfBuffer; //Stores known frames and offers a ROS service, "tf_frames"

  //provides an easy way to request and receive coordinate frame transform information.
  tf2_ros::TransformListener tfListener(tfBuffer); 
  ros::Publisher robot_velocity_publisher;
  ros::Subscriber fiducial_reader;
  geometry_msgs::Twist msg;

 //checking if the robot is explorer or the follower
  if(robot_name_.compare("explorer") == 0)
  {
    //building a velocity publisher 
    
    msg.linear.x = 0;
    msg.angular.z = 0.15;
    // move the bot with a constant angular velocity using node handle
    robot_velocity_publisher = robot_nh_.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 10);
   
  }
  
   
   robot_goal_.target_pose.header.frame_id = "map";
   robot_goal_.target_pose.header.stamp = ros::Time::now();
   robot_goal_.target_pose.pose.position.x = goal_loc.at(i).at(0);
   robot_goal_.target_pose.pose.position.y = goal_loc.at(i).at(1);
   robot_goal_.target_pose.pose.orientation.w = 1.0;
   i++;  

  while (!robot_client_.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up for "<< robot_name_);
  }
  
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) 
  {
      if (!goal_sent) {
      ROS_INFO_STREAM("Sending goal to "<<robot_name_);
      robot_client_.sendGoal(robot_goal_);//this should be sent only once per goal
      robot_client_.waitForResult();
      goal_sent = true;
    }
    if (robot_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
     
      if  (robot_goal_.target_pose.pose.position.x == -4)
      {
        ROS_INFO_STREAM( robot_name_<<" robot reached home.");
        break;
      }
  
      else
      {
       ROS_INFO_STREAM(robot_name_<<" robot reached goal" );
       if(robot_name_.compare("explorer") == 0)
      {
        
        ROS_INFO("Begin scan");
        fiducial_reader = robot_nh_.subscribe("/fiducial_transforms", 100, &acme::Robot::aruco_callback, this);
        while (saw_marker != true)
          {
           robot_velocity_publisher.publish(msg);
           ros::spinOnce();
          }
        fiducial_reader.shutdown();
        saw_marker = false;
        listen(tfBuffer);
      }
       
       goal_sent = false;
       
       robot_goal_.target_pose.header.frame_id = "map";
       robot_goal_.target_pose.header.stamp = ros::Time::now();
       robot_goal_.target_pose.pose.position.x = goal_loc.at(i).at(0);
       robot_goal_.target_pose.pose.position.y = goal_loc.at(i).at(1);
       robot_goal_.target_pose.pose.orientation.w = 1.0;
       i++;
      }  
     
    }
  
    loop_rate.sleep();
  }
 
}