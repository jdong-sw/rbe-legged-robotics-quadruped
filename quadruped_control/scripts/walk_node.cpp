#include <iostream>
#include "quadruped_control/MoveAction.h"
#include "quadruped_control/SetJointAction.h"
#include "actionlib/client/simple_action_client.h"

class WalkNode
{
public:
    WalkNode(ros::NodeHandle *node) : client("gait_controller", true)
    {
        this->node = node;

        ROS_INFO("Waiting for Move Server...");
        this->client.waitForServer(ros::Duration(30));

        ROS_INFO("Sending gait goal...");
        quadruped_control::MoveGoal goal;
        goal.distance = 1;
        goal.gaitType = 0;

        this->client.sendGoal(goal,
            boost::bind(&WalkNode::resultCB, this, _1, _2),
            boost::bind(&WalkNode::activeCB, this),
            boost::bind(&WalkNode::feedbackCB, this, _1));
        
    }

    ~WalkNode()
    {
        
    }

    void activeCB()
    {
    }

    void feedbackCB(const quadruped_control::MoveFeedback::ConstPtr &feedback)
    {
    }

    void resultCB(const actionlib::SimpleClientGoalState &state,
                  const quadruped_control::MoveResult::ConstPtr &gaitResult)
    {
    }

private:

    ros::NodeHandle *node;
    actionlib::SimpleActionClient<quadruped_control::MoveAction> client;
    quadruped_control::MoveFeedback actionFeedback;
    quadruped_control::MoveResult actionResult;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting walk node...");
    ros::init(argc, argv, "walk_node");
    ros::NodeHandle node;
    ROS_INFO("Initialized ros...");

    WalkNode walkNode(&node);
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
