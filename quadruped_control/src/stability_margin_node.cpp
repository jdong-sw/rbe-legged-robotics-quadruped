#include "ros/ros.h"
#include "std_msgs/Float64.h"

class StabilityMarginNode
{
public:
    StabilityMarginNode(ros::NodeHandle* node)
    {
        this->node = node;

        ROS_INFO("Subscribing to force sensor topics...");
        this->frSensorSubscriber = node->subscribe("/tibia_fr_force", 1, &StabilityMarginNode::frSensorCB, this);
        this->flSensorSubscriber = node->subscribe("/tibia_fl_force", 1, &StabilityMarginNode::flSensorCB, this);
        this->brSensorSubscriber = node->subscribe("/tibia_br_force", 1, &StabilityMarginNode::brSensorCB, this);
        this->blSensorSubscriber = node->subscribe("/tibia_bl_force", 1, &StabilityMarginNode::blSensorCB, this);

        ROS_INFO("Initializing force publishers...");
        this->stabilityPublisher = node->advertise<std_msgs::Float64>("/stability/ffsm", 1);

        ROS_INFO("Publishing stability...");
        timer = node->createTimer(ros::Duration(1.0/40.0),
                        std::bind(&StabilityMarginNode::publishStability, this));
    }

    ~StabilityMarginNode()
    {
        this->node->shutdown();
    }

    void publishStability()
    {
        // Get non-zero forces
        std::vector<double> forces;
        if (frForce > 0) forces.push_back(frForce);
        if (flForce > 0) forces.push_back(flForce);
        if (brForce > 0) forces.push_back(brForce);
        if (blForce > 0) forces.push_back(blForce);
        if (forces.size() < 3)
        {
            ffsmMsg.data = 0;
            stabilityPublisher.publish(ffsmMsg);
            return;
        }

        // Calculate average force
        double favg = 0;
        for (int i = 0; i < forces.size(); ++i)
        {
            favg += forces[i];
        }
        favg /= forces.size();

        // Calculate ffsm
        double ffsm = 1;
        for (int i = 0; i < forces.size(); ++i)
        {
            ffsm *= (forces[i] / favg);
        }

        ffsmMsg.data = ffsm;
        stabilityPublisher.publish(ffsmMsg);
    }


private:
    void frSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        frForce = msg->data;
    }

    void flSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        flForce = msg->data;
    }

    void brSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        brForce = msg->data;
    }

    void blSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        blForce = msg->data;
    }

    ros::NodeHandle* node;
    ros::ServiceClient linkStateClient;
    ros::Subscriber frSensorSubscriber;
    ros::Subscriber flSensorSubscriber;
    ros::Subscriber brSensorSubscriber;
    ros::Subscriber blSensorSubscriber;
    ros::Publisher stabilityPublisher;
    std_msgs::Float64 ffsmMsg;
    ros::Timer timer;
    double frForce;
    double flForce;
    double brForce;
    double blForce;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Stability Margin Node...");
    ros::init(argc, argv, "stability_margin_node");
    ROS_INFO("Initialized ros...");

    ros::NodeHandle node;
    StabilityMarginNode stabilityMarginNode(&node);

    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
