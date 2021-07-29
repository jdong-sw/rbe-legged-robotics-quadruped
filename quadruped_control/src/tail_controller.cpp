#include "actionlib/server/simple_action_server.h"
#include "quadruped_control/TailAction.h"
#include "quadruped_control/VectorOfPoints.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TailController
{
public:
    TailController(ros::NodeHandle node)
    {
        this->node = node;

        ROS_INFO("Subscribing to IMU topic");
        this->IMUSubscriber = node.subscribe("/imu", 10, &TailController::IMUCB, this);
        this->gainSubscriber = node.subscribe("/quadruped/stabilizer/gain", 10, &TailController::GainCB, this);
        this->offsetSubscriber = node.subscribe("/quadruped/stabilizer/offset", 10, &TailController::OffsetCB, this);

        ROS_INFO("Connecting to Tail Joint Controllers");
        this->tailYawPublisher = node.advertise<std_msgs::Float64>("/quadruped/tail/yaw_joint_position_controller/command", 1);
        this->tailPitchPublisher = node.advertise<std_msgs::Float64>("/quadruped/tail/pitch_joint_position_controller/command", 1);

        ROS_INFO("Starting Tail controller...");
        timer = node.createTimer(ros::Duration(1.0 / 50.0),
                        std::bind(&TailController::Stabilize, this));
        
        ROS_INFO("Tail controller ready.");
    }

    ~TailController()
    {
        this->node.shutdown();
    }

    void Stabilize()
    {
        // Get current orientation of body
        auto orientation = imu.orientation;

        // Convert to rpy
        tf2::Quaternion q;
        q.setX(orientation.x);
        q.setY(orientation.y);
        q.setZ(orientation.z);
        q.setW(orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);

        // Move the tail in the direction weightVector
        std_msgs::Float64 tailYawCommand, tailPitchCommand;
        tailYawCommand.data = -1*gain*r;
        tailPitchCommand.data = -1*gain*p + offset;
        tailYawPublisher.publish(tailYawCommand);
        tailPitchPublisher.publish(tailPitchCommand);
    }

    void IMUCB(const sensor_msgs::ImuConstPtr& msg)
    {
        this->imu = *msg.get();
    }

    void GainCB(const std_msgs::Float64ConstPtr& msg)
    {
        this->gain = msg->data;
        ROS_INFO("Gain changed to: %f", this->gain);
    }

    void OffsetCB(const std_msgs::Float64ConstPtr& msg)
    {
        this->offset = msg->data;
        ROS_INFO("Offset changed to: %f", this->offset);
    }

private:
    ros::NodeHandle node;
    ros::Publisher tailYawPublisher;
    ros::Publisher tailPitchPublisher;
    ros::Subscriber IMUSubscriber;
    ros::Subscriber gainSubscriber;
    ros::Subscriber offsetSubscriber;
    sensor_msgs::Imu imu;
    ros::Timer timer;
    double gain;
    double offset;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Tail Stability Compensator Node...");
    ros::init(argc, argv, "tail_controller_node");
    ROS_INFO("Initialized ros...");

    ros::NodeHandle node;
    TailController tail(node);
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}