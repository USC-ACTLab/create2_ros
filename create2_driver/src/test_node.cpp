#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  geometry_msgs::Twist msg;
  msg.linear.x = 0.1;
  msg.linear.y = -0.1;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
