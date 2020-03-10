#include "ros/ros.h"
#include "ros/console.h"
#include "controller.hpp"

int main(int argc, char **argv)
{
	// Initialize ros
    ros::init(argc, argv, "control_solution_node");

	// Create an instance of the node handler
    ros::NodeHandle nh;

	// Create an instance of the controller using the node handler
	Controller controller(nh);

  ros::Rate rate(1);

//let ros spin
  while(ros::ok()){
		ros::spin();
	}
}
