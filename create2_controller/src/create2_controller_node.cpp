#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <create2_controller/Goal.h>

class Controller
{
public:

    Controller(
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_goal()
        , m_subscribeGoal()
    {
        ros::NodeHandle nh;
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
        m_goal.arrival = ros::Time::now() + ros::Duration(10.0);
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
        float dt = e.current_real.toSec() - e.last_real.toSec();

        tf::StampedTransform transform;

        try {
            m_listener.lookupTransform("/odom", m_frame, ros::Time(0), transform);
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
        double relAngle = atan2(sin(targetAngle - posYaw), cos(targetAngle - posYaw));
        if (dist < 0.05) {
            relAngle = 0;
            dist = 0;
        }

        double v_left  = -relAngle * 2.0 + 20.0 * std::min(dist, 0.1) / (fabs(relAngle) + 1);
        double v_right =  relAngle * 2.0 + 20.0 * std::min(dist, 0.1) / (fabs(relAngle) + 1);

        ros::Duration timeToGo = m_goal.arrival - ros::Time::now();
        double v_desired = 0.1;//dist / timeToGo.toSec();

        double v_avg = (fabs(v_left) + fabs(v_right)) / 2.0;
        if (v_avg > 0.05) {
            v_left  = v_left  / v_avg * v_desired;
            v_right = v_right / v_avg * v_desired;
        }
        // ROS_INFO("Dist: %f, ttogo: %f, yaw: %f; %f", dist, timeToGo.toSec(), yaw, relAngle);
        // ROS_INFO("vavg: %f, vd: %f, vl: %f, vr: %f", v_avg, v_desired, v_left, v_right);

        geometry_msgs::Twist msg;
        msg.linear.x = v_left;
        msg.linear.y = v_right;
        //msg.linear.x = (v_left + v_right) / 2.0;
        //msg.angular.x = -v_left + msg.linear.x;
        m_pubNav.publish(msg);
    }

private:
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    create2_controller::Goal m_goal;
    ros::Subscriber m_subscribeGoal;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n("~");
  std::string frame;
  n.getParam("frame", frame);
  std::cout << frame << std::endl;

  Controller controller(frame, n);

  double frequency;
  n.param("frequency", frequency, 10.0);
  controller.run(frequency);

  // std::thread t(&Controlle::run, &controller);
  // t.detach();

  // ros::spin();

  return 0;
}
