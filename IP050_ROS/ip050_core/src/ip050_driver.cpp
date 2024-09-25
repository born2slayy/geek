#include <ros/ros.h>
#include <ros/console.h>
#include <ip050_core/ip050_driver.hpp>

using namespace aidl;

int main (int argc, char **argv) {
    ros::init(argc, argv, "ip050_driver");
    ros::NodeHandle nh;
    ip050BodyNode ip050 = ip050BodyNode(&nh);

	ros::AsyncSpinner spinner(0);
    spinner.start();
	
    ROS_INFO("ip050 Driver Node is started");
	ros::waitForShutdown();
}