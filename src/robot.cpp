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