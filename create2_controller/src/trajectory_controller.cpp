#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <create2_controller/TrajectoryState2D.h>

#include <control_toolbox/pid.h>

class Controller
{
public:

    Controller(
        const std::string& frame,
        const std::string& parentFrame)
        : m_frame(frame)
        , m_parentFrame(parentFrame)
        , m_pubCmdVel()
        , m_listener()
        , m_subDesiredState()
        // , m_pidTheta()
        // , m_pidVelocity()
        // , m_lastPosX()
        // , m_lastPosY()
    {
        ros::NodeHandle nh;

        // m_pidTheta.initPid(0.1, 0.05, 0.0, 0.0, 0.0);
        // m_pidVelocity.initPid(0.5, 0.0, 0.0, 0.0, 0.0);

        m_pubCmdVel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subDesiredState = nh.subscribe("desired_state", 1, &Controller::onDesiredState, this);
    }

    // void run(double frequency)
    // {
    //     ros::NodeHandle node;
    //     ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
    //     ros::spin();
    // }

private:
    void onDesiredState(
        const create2_controller::TrajectoryState2D::ConstPtr& msg)
    {
        // We execute an update step each time a new desired state arrives
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    double clamp(double value, double min, double max)
    {
        return std::min(std::max(value, min), max);
    }

private:
    std::string m_frame;
    std::string m_parentFrame;
    ros::Publisher m_pubCmdVel;
    tf::TransformListener m_listener;
    ros::Subscriber m_subDesiredState;
    // control_toolbox::Pid m_pidTheta;
    // control_toolbox::Pid m_pidVelocity;
    // double m_lastPosX;
    // double m_lastPosY;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_controller");

  ros::NodeHandle n("~");
  std::string frame;
  std::string parentFrame;
  n.getParam("frame", frame);
  n.param<std::string>("parentFrame", parentFrame, "/world");

  Controller controller(frame, parentFrame);

  // double frequency;
  // n.param("frequency", frequency, 10.0);
  // controller.run(frequency);

  // std::thread t(&Controlle::run, &controller);
  // t.detach();

  // ros::spin();

  return 0;
}
