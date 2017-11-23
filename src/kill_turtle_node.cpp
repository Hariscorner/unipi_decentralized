#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "my_shadowkiller_node");
	ros::NodeHandle nh;
	ros::service::waitForService("kill");
	ros::ServiceClient killerBose = nh.serviceClient<turtlesim::Kill>("kill");
	
	//kill the default turtle
	turtlesim::Kill::Request req_turtle;
	turtlesim::Kill::Response resp_turtle;
	req_turtle.name="turtle1";
	killerBose.call(req_turtle,resp_turtle);
	
}