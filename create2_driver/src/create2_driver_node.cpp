#include <ros/ros.h>
#include <signal.h>
#include "geometry_msgs/Twist.h"

#include <create2_cpp/Create2.h>

class Create2ROS
  : public Create2
{
public:
  Create2ROS(
    const std::string& port,
    uint32_t brcPin,
    bool useBrcPin)
    : subscribeCmdVel_()
    , Create2(port, brcPin, useBrcPin)
  {
    ros::NodeHandle n;
    start();
    safe();

    startStream({
      Create2::SensorOIMode,
      Create2::SensorVoltage,
      Create2::SensorCurrent,
      Create2::SensorTemperature,
      Create2::SensorBatteryCharge,
      Create2::SensorBatteryCapacity,
      Create2::SensorCliffLeftSignal,
      Create2::SensorCliffFrontLeftSignal,
      Create2::SensorCliffFrontRightSignal,
      Create2::SensorCliffRightSignal,
      Create2::SensorLeftEncoderCounts,
      Create2::SensorRightEncoderCounts,
    });

    // ROS_INFO("Battery: %.1f %%", create2_.batteryCharge() / (float)create2_.batteryCapacity() * 100.0f);


    digitsLedsAscii("ABCD");

    subscribeCmdVel_ = n.subscribe("cmd_vel", 1, &Create2ROS::cmdVelChanged, this);
  }

  ~Create2ROS()
  {
    std::cout << "destruct createROS" << std::endl;
    power();
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    driveDirect(msg->linear.y * 1000, msg->linear.x * 1000);
  }

  virtual void onUpdate(
    const State& state)
  {
    std::cout << "Mode: " << state.mode << std::endl;
    std::cout << "V: " << state.voltageInMV << " mV" << std::endl;
    std::cout << "Current: " << state.currentInMA << " mA" << std::endl;
    std::cout << "Temp: " << (int)state.temperatureInDegCelcius << " degC" << std::endl;
    std::cout << "Charge: " << state.batteryChargeInMAH << " mAh" << std::endl;
    std::cout << "Capacity: " << state.batteryCapacityInMAH << " mAh" << std::endl;
    std::cout << "CliffLeft: " << state.cliffLeftSignalStrength << std::endl;
    std::cout << "CliffFrontLeft: " << state.cliffFrontLeftSignalStrength << std::endl;
    std::cout << "CliffFrontRight: " << state.cliffFrontRightSignalStrength << std::endl;
    std::cout << "CliffRight: " << state.cliffRightSignalStrength << std::endl;
    std::cout << "LeftEncoder: " << state.leftEncoderCounts << std::endl;
    std::cout << "RightEncoder: " << state.rightEncoderCounts << std::endl;

  }

private:
  ros::Subscriber subscribeCmdVel_;
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

  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    g_create2->update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
