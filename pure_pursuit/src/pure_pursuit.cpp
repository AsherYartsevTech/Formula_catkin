#include "ros/ros.h"
#include "pure_pursuit_controller.h"
#include <iostream>
#include "custom_msgs/pp_msg.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pure_pursuit");

	std::vector<pair<double,double>> wps;
	wps.push_back(std::make_pair(0,0));
	wps.push_back(std::make_pair(10,10));
	wps.push_back(std::make_pair(20,0));

	PurePursuitController controller = PurePursuitController(7.0, 10.0,wps);

	ros::NodeHandle n;

	ros::Publisher pp_pub = n.advertise<custom_msgs::pp_msg>("pp_controller", 10);
   	  	std::cout << "before while" << std::endl;

  	ros::Rate loop_rate(5);
	while(ros::ok()){	
		custom_msgs::pp_msg msg;
		msg.gas = 0.5;
		msg.steering = -0.2;
		ROS_INFO("%lf, %lf", msg.gas, msg.steering);

		pp_pub.publish(msg);

		loop_rate.sleep();
	}
	
	pair<double,double> ld_point;
	controller.calc_ld_point(std::make_pair(3.5,4.7),ld_point);
	// std::cout << ld_point.first << ", " << ld_point.second << std::endl;
	 
	return 0;
}