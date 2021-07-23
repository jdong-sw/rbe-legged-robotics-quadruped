#include "actionlib/server/simple_action_server.h"
#include "quadruped_control/SetJointAction.h"
#include "quadruped_control/SolveFKPose.h"
#include "quadruped_control/SolveIKPose.h"
#include "quadruped_control/Pose.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class SetJointAction
{
public:
    SetJointAction(std::string name) :
        server(node, name, boost::bind(&SetJointAction::executeCB, this, _1), false),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Subscribing to Joint States...");
        this->jointStateSubscriber = node.subscribe("/quadruped/joint_states", 10, &SetJointAction::jointStatesCB, this);

        ROS_INFO("Publishing to Joint Controllers");
        this->tailYawJointPublisher = node.advertise<std_msgs::Float64>("/quadruped/tail/yaw_joint_position_controller/command", 1);
        this->tailPitchJointPublisher = node.advertise<std_msgs::Float64>("/quadruped/tail/pitch_joint_position_controller/command", 1);

        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<quadruped_control::SolveFKPose>("/quadruped/tail/fk");

        ROS_INFO("Starting...");
        server.start();
    }

    ~SetJointAction()
    {
        this->node.shutdown();
    }

    void executeCB(const quadruped_control::SetJointGoalConstPtr &goal)
    {
        // Set goal
        this->yawTarget = goal->goal[0];
        this->pitchTarget = goal->goal[1];
        this->eps = goal->eps;

        // Command joints
        std_msgs::Float64 tailYawJointCommand, tailPitchJointCommand;
        tailYawJointCommand.data = this->yawTarget;
        tailPitchJointCommand.data = this->pitchTarget;
        tailYawJointPublisher.publish(tailYawJointCommand);
        tailPitchJointPublisher.publish(tailPitchJointCommand);

        // Get current time
        double start = ros::Time::now().toSec();

        // Make sure goal state is not current state
        if (calculateJointError() < this->eps)
        {
            this->actionResult.result = currentState.position;
            this->actionResult.error = calculateJointError();
            this->actionResult.time = 0;
            server.setSucceeded();
            return;
        }

        // Wait for joints to start moving
        ros::Rate rate(20);
        while(isRobotIdle())
        {
            rate.sleep();
        }

        // Publish feedback
        while (!isRobotIdle() && server.isActive())
        {
            // Check for preemption
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Joint action preempted.");
                server.setPreempted();
                return;
            }

            this->actionFeedback.positions = currentState.position;
            this->actionFeedback.error = calculateJointError();
            this->actionFeedback.time = ros::Time::now().toSec() - start;
            server.publishFeedback(this->actionFeedback);

            rate.sleep();
        }

        // Publish result
        this->actionResult.result = currentState.position;
        this->actionResult.error = calculateJointError();
        this->actionResult.time = ros::Time::now().toSec() - start;
        if (this->actionResult.error < eps)
        {
            this->actionResult.errorCode = 0;
        }
        else
        {
            this->actionResult.errorCode = -2;
        }
        
        server.setSucceeded(this->actionResult);
    }

    quadruped_control::SolveFKPoseResponse getCurrentPose()
    {
        // Send FK request to service
        quadruped_control::SolveFKPose fkMsg;
        fkMsg.request.jointPositions = this->currentState.position;
        fkClient.call(fkMsg);

        return fkMsg.response;

    }

    double calculateJointError()
    {
        double yawError = this->yawTarget - currentState.position[0];
        double pitchError = this->pitchTarget - currentState.position[1];
        return sqrt(pow(yawError, 2) + pow(pitchError, 2));
    }

    bool isRobotIdle()
    {
        std::vector<double> velocity = this->currentState.velocity;
        double magnitude = 0;
        for (int i = 0; i < velocity.size(); ++i)
        {
            magnitude += pow(velocity[i], 2);
        }

        double jointError = calculateJointError();

        return magnitude < 0.01 && jointError < eps;
    }

    void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
    {
        this->temp = *msg.get();
        int yawIndex, pitchIndex;
        for (int i = 0; i < temp.name.size(); ++i)
        {
            std::string name_i = temp.name[i];
            if (name_i.find("tail") != std::string::npos)
            {
                if (name_i.find("yaw") != std::string::npos)
                {
                    yawIndex = i;
                }
                else if (name_i.find("pitch") != std::string::npos)
                {
                    pitchIndex = i;
                }
            }
        }
        
        this->currentState.name = {temp.name[yawIndex], temp.name[pitchIndex]};
        this->currentState.position = {temp.position[yawIndex], temp.position[pitchIndex]};
        this->currentState.velocity = {temp.velocity[yawIndex], temp.velocity[pitchIndex]};
        this->currentState.effort = {temp.effort[yawIndex], temp.effort[pitchIndex]};
    }

private:
    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<quadruped_control::SetJointAction>  server;
    quadruped_control::SetJointFeedback actionFeedback;
    quadruped_control::SetJointResult actionResult;
    ros::ServiceClient fkClient;
    ros::Subscriber jointStateSubscriber;
    ros::Publisher tailYawJointPublisher;
    ros::Publisher tailPitchJointPublisher;
    sensor_msgs::JointState currentState;
    sensor_msgs::JointState temp;
    double yawTarget;
    double pitchTarget;
    double eps;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Joint Action Server...");
    ros::init(argc, argv, "tail_joint_action");

    SetJointAction actionServer("tail_joint_action");
    ros::spin();
    return 0;
}
