#include <ros/ros.h>
#include <signal.h>
#include "geometry_msgs/Twist.h"

#include <create2_cpp/Create2.h>

class Create2ROS
{
public:
  Create2ROS(
    const std::string& port,
    uint32_t brcPin,
    bool useBrcPin)
    : subscribeCmdVel_()
    , create2_(port, brcPin, useBrcPin)
  {
    ros::NodeHandle n;
    create2_.start();
    create2_.safe();

    ROS_INFO("Battery: %.1f %%", create2_.batteryCharge() / (float)create2_.batteryCapacity() * 100.0f);


    create2_.digitsLedsAscii("ABCD");

    subscribeCmdVel_ = n.subscribe("cmd_vel", 1, &Create2ROS::cmdVelChanged, this);
  }

  ~Create2ROS()
  {
    std::cout << "destruct createROS" << std::endl;
    create2_.power();
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    create2_.driveDirect(msg->linear.y * 1000, msg->linear.x * 1000);
  }

private:
  ros::Subscriber subscribeCmdVel_;
  Create2 create2_;
};


Create2ROS* g_create2;

void mySigintHandler(int sig)
{
  // this will disconnect from the create and make sure that
  // it is not in full mode (where it would drain the battery)
  delete g_create2;

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create2_driver_node");

  ros::NodeHandle nh("~");

  std::string port;
  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  int brcPin;
  nh.param<int>("brcPin", brcPin, 87);
  bool useBrcPin;
  nh.param<bool>("useBrcPin", useBrcPin, false);

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  g_create2 = new Create2ROS(port, brcPin, useBrcPin);

  ros::spin();

  return 0;
}
