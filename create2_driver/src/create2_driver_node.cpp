#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <create2_cpp/Create2.h>

class Create2ROS
  : public Create2
{
public:
  Create2ROS(
    const std::string& port,
    uint32_t brcPin,
    bool useBrcPin)
    : Create2(port, brcPin, useBrcPin)
    , subscribeCmdVel_()
    , br_()
    , odomPub_()
    , hasPreviousCounts_(false)
    , previousLeftEncoderCount_(0)
    , previousRightEncoderCount_(0)
    , x_(0)
    , y_(0)
    , theta_(0)
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
    odomPub_ = n.advertise<nav_msgs::Odometry>("odom", 50);
  }

  ~Create2ROS()
  {
    std::cout << "destruct createROS" << std::endl;
    digitsLedsAscii("    ");
    // power();
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    driveDirect(msg->linear.y * 1000, msg->linear.x * 1000);
  }

  virtual void onUpdate(
    const State& state)
  {
    static ros::Time lastTime = ros::Time::now();

    ros::Time currentTime = ros::Time::now();

    // double dt = (currentTime - lastTime).toSec();
    double dt = 0.015; // in streaming mode this should be constant; avoid measuring since that causes errors introduced by
    // communication delays etc.
    double lastX = x_;
    double lastY = y_;
    double lastTheta = theta_;
    double velocity; // translational
    double angularVelocity;
#if 0
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
#endif
    if (hasPreviousCounts_)
    {
      int32_t dtl = state.leftEncoderCounts - previousLeftEncoderCount_;
      int32_t dtr = state.rightEncoderCounts - previousRightEncoderCount_;

      // std::cout << "dtl: " << dtl << " dtr: " << dtr << std::endl;

      if (dtl < -30000) {
        dtl += 65535;
      }
      if (dtl > 30000) {
        dtl -= 65535;
      }
      if (dtr < -30000) {
        dtr += 65535;
      }
      if (dtr > 30000) {
        dtr -= 65535;
      }

      double Dl = M_PI * WheelDiameterInMM * dtl / CountsPerRev;
      double Dr = M_PI * WheelDiameterInMM * dtr / CountsPerRev;
      double Dc = (Dl + Dr) / 2.0;

      // std::cout << "Dl: " << Dl << " Dr: " << Dr << " Dc: " << Dc << std::endl;

      x_ += Dc * cos(theta_) / 1000.0;
      y_ += Dc * sin(theta_) / 1000.0;
      theta_ = fmod(theta_ + (Dr - Dl) / WheelDistanceInMM, 2 * M_PI);
      if (theta_ < 0) {
        theta_ += 2 * M_PI;
      }

      velocity = dt > 0 ? ((Dc / 1000.0) / dt) : 0.0;
      angularVelocity = dt > 0 ? ((Dr - Dl) / WheelDistanceInMM / dt) : 0.0;
      // std::cout << "dt: " << dt << " vel: " << velocity << " angVel: " << angularVelocity << std::endl;
    }

    previousLeftEncoderCount_ = state.leftEncoderCounts;
    previousRightEncoderCount_ = state.rightEncoderCounts;
    hasPreviousCounts_ = true;

    // std::cout << "State: (" << x_ << ", " << y_ << ", " << theta_ << ")" << std::endl;

    // send tf
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_, y_, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, theta_);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, currentTime, "odom", "base_link"));

    // publish odomotry message
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta_);

    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dt > 0 ? (x_ - lastX)/dt : 0.0;
    odom.twist.twist.linear.y = dt > 0 ? (y_ - lastY)/dt : 0.0;
    odom.twist.twist.linear.z = velocity; // hack...; Maybe add another message just for velocities?
    odom.twist.twist.angular.z = angularVelocity;

    //publish the message
    odomPub_.publish(odom);

    // update display
    char buf[4] = {' ', ' ', ' ', ' '};
    std::snprintf(buf, 4, "%d", (int)(state.batteryChargeInMAH / (float)state.batteryCapacityInMAH * 100.0));
    digitsLedsAscii(buf);

    lastTime = currentTime;

  }

private:
  ros::Subscriber subscribeCmdVel_;
  tf::TransformBroadcaster br_;
  ros::Publisher odomPub_;

  bool hasPreviousCounts_;
  int16_t previousLeftEncoderCount_;
  int16_t previousRightEncoderCount_;

  double x_;
  double y_;
  double theta_;
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

  while (ros::ok())
  {

    g_create2->update();

    ros::spinOnce();
    ros::Duration(0.001).sleep(); // 1ms
  }

  return 0;
}
