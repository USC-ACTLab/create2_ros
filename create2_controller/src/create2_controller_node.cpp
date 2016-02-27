#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <create2_controller/Goal.h>

#include <control_toolbox/pid.h>

class Controller
{
public:

    Controller(
        const std::string& frame,
        const std::string& parentFrame)
        : m_frame(frame)
        , m_parentFrame(parentFrame)
        , m_pubNav()
        , m_listener()
        , m_goal()
        , m_subscribeGoal()
        , m_pidTheta()
        , m_pidVelocity()
        , m_lastPosX()
        , m_lastPosY()
    {
        ros::NodeHandle nh;

        m_pidTheta.initPid(0.1, 0.05, 0.0, 0.0, 0.0);
        m_pidVelocity.initPid(0.5, 0.0, 0.0, 0.0, 0.0);

        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const create2_controller::Goal::ConstPtr& msg)
    {
        m_goal = *msg;
        // TODO: HACK HACK
        // m_goal.arrival = ros::Time::now() + ros::Duration(10.0);
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void iteration(const ros::TimerEvent& e)
    {

        if (m_goal.arrival == ros::Time(0)) {
            // We didn't start yet or we are done.
            geometry_msgs::Twist msg;
            m_pubNav.publish(msg);
            return;
        }

        ros::Duration dtRos = e.current_real - e.last_real;
        float dt = e.current_real.toSec() - e.last_real.toSec();

        tf::StampedTransform transform;

        try {
            m_listener.lookupTransform(m_parentFrame, m_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return;
        }

        tfScalar roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

        double goalX = m_goal.x;
        double goalY = m_goal.y;
        double posX = transform.getOrigin().x();
        double posY = transform.getOrigin().y();
        double posYaw = yaw;

        double dist = sqrt(pow(posX - goalX, 2) + pow(posY - goalY, 2));
        double targetAngle = atan2(goalY - posY, goalX - posX);
        double errorTheta = atan2(sin(targetAngle - posYaw), cos(targetAngle - posYaw));

        ros::Duration timeToGo = m_goal.arrival - ros::Time::now();
        double targetVelocity = std::max(dist / timeToGo.toSec(), 0.0);
        double velocity = sqrt(pow(posX - m_lastPosX, 2) + pow(posY - m_lastPosY, 2)) / dt;

        double errorVelocity = targetVelocity - velocity;

        double outputVelocity = clamp(targetVelocity+m_pidVelocity.computeCommand(errorVelocity, dtRos), 0, 0.5);//clamp(m_pidVelocity.updatePid(errorVelocity, dtRos), 0, 0.5);
        double outputTheta = clamp(m_pidTheta.computeCommand(errorTheta, dtRos), -0.2, 0.2);
        // if (outputVelocity < 0.01) {
        //     outputTheta = 0;
        //     outputVelocity = 0;
        // }


        geometry_msgs::Twist msg;
        msg.linear.x = clamp(outputVelocity - outputTheta, -0.5, 0.5);
        msg.linear.y = clamp(outputVelocity + outputTheta, -0.5, 0.5);

        // ROS_INFO("Vel: is: %f, target: %f output: %f", velocity, targetVelocity, outputVelocity);
        ROS_INFO("dist: %f, %f; %f, %f", dist, targetVelocity, velocity, outputVelocity);
        // ROS_INFO("Vel: %f, %f", msg.linear.x, msg.linear.y);
        //msg.linear.x = (v_left + v_right) / 2.0;
        //msg.angular.x = -v_left + msg.linear.x;
        m_pubNav.publish(msg);

        m_lastPosX = posX;
        m_lastPosY = posY;
    }

    double clamp(double value, double min, double max)
    {
        return std::min(std::max(value, min), max);
    }

private:
    std::string m_frame;
    std::string m_parentFrame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    create2_controller::Goal m_goal;
    ros::Subscriber m_subscribeGoal;
    control_toolbox::Pid m_pidTheta;
    control_toolbox::Pid m_pidVelocity;
    double m_lastPosX;
    double m_lastPosY;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n("~");
  std::string frame;
  std::string parentFrame;
  n.getParam("frame", frame);
  n.param<std::string>("parentFrame", parentFrame, "/world");
  std::cout << frame << std::endl;

  Controller controller(frame, parentFrame);

  double frequency;
  n.param("frequency", frequency, 10.0);
  controller.run(frequency);

  // std::thread t(&Controlle::run, &controller);
  // t.detach();

  // ros::spin();

  return 0;
}
