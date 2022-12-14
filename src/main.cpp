#include "../include/robot.hpp"
#include<vector>


int main(int argc, char** argv)
{
//declaring goal arrays
std::vector<std::vector<double>> explorer_goals; 

// ros initialization
ros::init(argc, argv, "simple_navigation_goals");
ROS_INFO("STARTED EXPLORER");

//constructing explorer and follower objects
acme::Robot explorer("explorer");
//Moving explorer and follower
ROS_INFO("GETTING EXPLORER GOALS");
explorer_goals = explorer.get_goals();
ROS_INFO("GOT EXPLORER GOALS");
explorer.move_to_goal(explorer_goals);

//shutting down 
ros::shutdown();
}