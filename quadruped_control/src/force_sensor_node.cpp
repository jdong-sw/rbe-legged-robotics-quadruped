#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ContactState.h"
#include "tf2/LinearMath/Transform.h"

class ForceSensorNode
{
public:
    ForceSensorNode(ros::NodeHandle* node)
    {
        this->node = node;

        ROS_INFO("Subscribing to Gazebo GetLinkState service...");
        this->linkStateClient = node->serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Subscribing to bumper sensor topics...");
        this->frSensorSubscriber = node->subscribe("/tibia_fr_bumper", 1, &ForceSensorNode::frSensorCB, this);
        this->flSensorSubscriber = node->subscribe("/tibia_fl_bumper", 1, &ForceSensorNode::flSensorCB, this);
        this->brSensorSubscriber = node->subscribe("/tibia_br_bumper", 1, &ForceSensorNode::brSensorCB, this);
        this->blSensorSubscriber = node->subscribe("/tibia_bl_bumper", 1, &ForceSensorNode::blSensorCB, this);

        ROS_INFO("Initializing force publishers...");
        this->frForcePublisher = node->advertise<std_msgs::Float64>("/tibia_fr_force", 1);
        this->flForcePublisher = node->advertise<std_msgs::Float64>("/tibia_fl_force", 1);
        this->brForcePublisher = node->advertise<std_msgs::Float64>("/tibia_br_force", 1);
        this->blForcePublisher = node->advertise<std_msgs::Float64>("/tibia_bl_force", 1);
    }

    ~ForceSensorNode()
    {
        this->node->shutdown();
    }


private:
    tf2::Transform getForceOnLink(gazebo_msgs::ContactState state, std::string linkName)
    {
        // Get force vector
        auto force = state.total_wrench.force;

        // Get link orientation
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = linkName;
        linkStateMsg.request.reference_frame = "ground";
        bool success = linkStateClient.call(linkStateMsg);

        auto pose = linkStateMsg.response.link_state.pose;
        auto vector = vectorToTransform(force);
        auto transform = poseToTransform(pose);

        // Transform force vector
        tf2::Transform transformedForce;
        transformedForce.mult(transform, vector);

        return transformedForce;
    }

    void frSensorCB(const gazebo_msgs::ContactsStateConstPtr& msg)
    {
        // Publish 0 if no collisions
        if (msg->states.size() == 0)
        {
            frForceMsg.data = 0;
            frForcePublisher.publish(frForceMsg);
            return;
        }

        auto force = getForceOnLink(msg->states[0], "tibia_fr");

        // Publish z component
        frForceMsg.data = force.getOrigin().getZ();
        frForcePublisher.publish(frForceMsg);
    }

    void flSensorCB(const gazebo_msgs::ContactsStateConstPtr& msg)
    {
        // Publish 0 if no collisions
        if (msg->states.size() == 0)
        {
            flForceMsg.data = 0;
            flForcePublisher.publish(flForceMsg);
            return;
        }

        auto force = getForceOnLink(msg->states[0], "tibia_fl");

        // Publish z component
        flForceMsg.data = force.getOrigin().getZ();
        flForcePublisher.publish(flForceMsg);
    }

    void brSensorCB(const gazebo_msgs::ContactsStateConstPtr& msg)
    {
        // Publish 0 if no collisions
        if (msg->states.size() == 0)
        {
            brForceMsg.data = 0;
            brForcePublisher.publish(brForceMsg);
            return;
        }

        auto force = getForceOnLink(msg->states[0], "tibia_br");

        // Publish z component
        brForceMsg.data = force.getOrigin().getZ();
        brForcePublisher.publish(brForceMsg);
    }

    void blSensorCB(const gazebo_msgs::ContactsStateConstPtr& msg)
    {
        // Publish 0 if no collisions
        if (msg->states.size() == 0)
        {
            blForceMsg.data = 0;
            blForcePublisher.publish(blForceMsg);
            return;
        }

        auto force = getForceOnLink(msg->states[0], "tibia_bl");

        // Publish z component
        blForceMsg.data = force.getOrigin().getZ();
        blForcePublisher.publish(blForceMsg);
    }

    tf2::Transform poseToTransform(geometry_msgs::Pose pose)
    {
        tf2::Transform transform;
        tf2::Vector3 position;
        tf2::Quaternion rotation;

        position.setX(pose.position.x);
        position.setY(pose.position.y);
        position.setZ(pose.position.z);

        rotation.setX(pose.orientation.x);
        rotation.setY(pose.orientation.y);
        rotation.setZ(pose.orientation.z);
        rotation.setW(pose.orientation.w);

        transform.setOrigin(position);
        transform.setRotation(rotation);

        return transform;
    }

    tf2::Transform vectorToTransform(geometry_msgs::Vector3 vector)
    {
        tf2::Transform transform;
        tf2::Vector3 position;
        tf2::Quaternion rotation;

        position.setX(vector.x);
        position.setY(vector.y);
        position.setZ(vector.z);
        rotation.setRPY(0, 0, 0);

        transform.setOrigin(position);
        transform.setRotation(rotation);

        return transform;
    }


    geometry_msgs::Point GetPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "body";
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    ros::NodeHandle* node;
    ros::ServiceClient linkStateClient;
    ros::Subscriber frSensorSubscriber;
    ros::Subscriber flSensorSubscriber;
    ros::Subscriber brSensorSubscriber;
    ros::Subscriber blSensorSubscriber;
    ros::Publisher frForcePublisher;
    ros::Publisher flForcePublisher;
    ros::Publisher brForcePublisher;
    ros::Publisher blForcePublisher;
    std_msgs::Float64 frForceMsg;
    std_msgs::Float64 flForceMsg;
    std_msgs::Float64 brForceMsg;
    std_msgs::Float64 blForceMsg;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Force Sensor Node...");
    ros::init(argc, argv, "force_sensor_node");
    ROS_INFO("Initialized ros...");

    ros::NodeHandle node;
    ForceSensorNode forceSensorNode(&node);
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
