#include "ros/ros.h"
#include "project_1/vec_xyz.h"

void messageCallback(const project_1::vec_xyz::ConstPtr& msg)
{
ROS_INFO_STREAM("I heard:"<< msg->x << msg->y<< msg->z);
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "listener");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("point_xyz", 1, messageCallback);
ros::spin();
return 0;
}