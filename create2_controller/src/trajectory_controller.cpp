#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <create2_controller/TrajectoryState2D.h>

#include <Eigen/Core>

class Controller
{
public:

    Controller(
        const std::string& frame,
        const std::string& parentFrame,
        float Kx,
        float Ky,
        float Ktheta)
        : m_frame(frame)
        , m_parentFrame(parentFrame)
        , m_pubCmdVel()
        , m_listener()
        , m_subDesiredState()
        , m_pubPoseCurrent()
        , m_pubPoseDesired()
        , m_Kx(Kx)
        , m_Ky(Ky)
        , m_Ktheta(Ktheta)
    {
        ros::NodeHandle nh;

        m_pubCmdVel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subDesiredState = nh.subscribe("desired_state", 1, &Controller::onDesiredState, this);

        m_timer = nh.createTimer(ros::Duration(1.0), &Controller::onWatchdog, this);

        // for debugging/tuning
        m_pubPoseCurrent = nh.advertise<geometry_msgs::Pose2D>("pose_current", 1);
        m_pubPoseDesired = nh.advertise<geometry_msgs::Pose2D>("pose_desired", 1);
    }

private:
    void onDesiredState(
        const create2_controller::TrajectoryState2D::ConstPtr& msg)
    {
        // We execute an update step each time a new desired state arrives

        // Get current state
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
        double posX = transform.getOrigin().x();
        double posY = transform.getOrigin().y();

        // p_c = (x_c, y_c, theta_c): current posture
        Eigen::Vector3f p_c(posX, posY, yaw);
        geometry_msgs::Pose2D p_c_msg;
        p_c_msg.x = p_c(0);
        p_c_msg.y = p_c(1);
        p_c_msg.theta = p_c(2);
        m_pubPoseCurrent.publish(p_c_msg);

        // p_r = (x_r, y_r, theta_r): reference posture
        float desiredYaw = atan2(msg->velocity.y, msg->velocity.x);
        Eigen::Vector3f p_r(msg->position.x, msg->position.y, desiredYaw);
        geometry_msgs::Pose2D p_r_msg;
        p_r_msg.x = p_r(0);
        p_r_msg.y = p_r(1);
        p_r_msg.theta = p_r(2);
        m_pubPoseDesired.publish(p_r_msg);

        Eigen::Vector2f vel(msg->velocity.x, msg->velocity.y);
        float v_r = vel.norm();
        float w_r = 0.0;

        float velNormSquared = vel.squaredNorm();
        if (velNormSquared > 1e-6) {
            w_r = (msg->velocity.y * msg->acceleration.x - msg->velocity.x * msg->acceleration.y) / velNormSquared;
        }

        // p_e = (x_e, y_e, theta_e) = T_e(p_r - p_c): "difference" between p_r and p_c
        float theta_c = p_c(2);
        Eigen::Matrix3f T_e;
        T_e << cos(theta_c) , sin(theta_c), 0,
               -sin(theta_c), cos(theta_c), 0,
               0            , 0           , 1;
        Eigen::Vector3f p_e = T_e * (p_r - p_c);
        float x_e = p_e(0);
        float y_e = p_e(1);
        float theta_e = p_e(2);

        float v = v_r * cos(theta_e) + m_Kx * x_e;
        float w = w_r + v_r * (m_Ky * y_e + m_Ktheta * sin(theta_e));

        // apply control
        // ROS_INFO("v: %f, w: %f", v, w);

        // convert w [rad/s] => w_v [m/s]
        const float WheelDistanceInMM = 235.0;
        const float u = M_PI * WheelDistanceInMM / 1000.0; // circumference in m (~0.74)

        float w_v = w / (2 * M_PI) * u;

        if (v-w_v < -0.5 || v-w_v > 0.5 || v + w_v < -0.5 || v + w_v > 0.5) {
            ROS_WARN("Output Satuation!");
        }


        geometry_msgs::Twist msg_cmd_vel;
        msg_cmd_vel.linear.x = clamp(v - w_v, -0.5, 0.5);
        msg_cmd_vel.linear.y = clamp(v + w_v, -0.5, 0.5);
        m_pubCmdVel.publish(msg_cmd_vel);
    }

    double clamp(double value, double min, double max)
    {
        return std::min(std::max(value, min), max);
    }

    void onWatchdog(const ros::TimerEvent& e)
    {

    }

private:
    std::string m_frame;
    std::string m_parentFrame;
    ros::Publisher m_pubCmdVel;
    tf::TransformListener m_listener;
    ros::Subscriber m_subDesiredState;

    ros::Publisher m_pubPoseCurrent;
    ros::Publisher m_pubPoseDesired;

    float m_Kx;
    float m_Ky;
    float m_Ktheta;

    ros::Timer m_timer;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_controller");

  ros::NodeHandle n("~");
  std::string frame;
  std::string parentFrame;
  float Kx, Ky, Ktheta;
  n.getParam("frame", frame);
  n.param<std::string>("parentFrame", parentFrame, "/world");
  n.getParam("Kx", Kx);
  n.getParam("Ky", Ky);
  n.getParam("Ktheta", Ktheta);

  Controller controller(frame, parentFrame, Kx, Ky, Ktheta);

  ros::spin();

  return 0;
}
