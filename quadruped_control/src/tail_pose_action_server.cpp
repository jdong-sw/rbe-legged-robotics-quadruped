#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "quadruped_control/SetPoseAction.h"
#include "quadruped_control/SetJointAction.h"
#include "quadruped_control/SolveFKPose.h"
#include "quadruped_control/SolveIKPose.h"
#include "quadruped_control/Pose.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class SetPoseAction
{
public:
    SetPoseAction(std::string name) :
        server(node, name, boost::bind(&SetPoseAction::executeCB, this, _1), false),
        client("tail_joint_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Joint State Server...");
        this->client.waitForServer(ros::Duration(30));

        ROS_INFO("Subscribing to Joint States...");
        this->jointStateSubscriber = node.subscribe("/quadruped/joint_states", 10, &SetPoseAction::jointStatesCB, this);

        ROS_INFO("Subscribing to IKPoseSolver service...");
        this->ikClient = node.serviceClient<quadruped_control::SolveIKPose>("/quadruped/tail/ik");

        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<quadruped_control::SolveFKPose>("/quadruped/tail/fk");

        ROS_INFO("Starting...");
        server.start();
    }

    ~SetPoseAction()
    {
        this->node.shutdown();
    }

    void executeCB(const quadruped_control::SetPoseGoalConstPtr &goal)
    {
        // Run IK to get joint positions
        quadruped_control::SolveIKPose ikMsg;
        ikMsg.request.initialState = this->currentState.position;
        ikMsg.request.goal = goal->goal;

        this->ikClient.call(ikMsg);

        // IK Solver failed
        if (ikMsg.response.result < 0)
        {
            this->actionResult.errorCode = -1;
            server.setSucceeded(this->actionResult);
        }
        this->targetx = goal->goal.x;
        this->targety = goal->goal.y;
        this->targetz = goal->goal.z;
        this->yawTarget = ikMsg.response.solution[0];
        this->pitchTarget = ikMsg.response.solution[1];
        this->eps = goal->eps;
        this->actionFeedback.target = goal->goal;

        // Send joint positions to joint action client
        quadruped_control::SetJointGoal jointAction;
        jointAction.goal = ikMsg.response.solution;
        jointAction.eps = goal->eps;
        this->client.sendGoal(jointAction,
            boost::bind(&SetPoseAction::publishResult, this, _1, _2),
            boost::bind(&SetPoseAction::activeCB, this),
            boost::bind(&SetPoseAction::publishFeedback, this, _1));

        // Get current time
        double start = ros::Time::now().toSec();

        // Wait for joints to start moving
        ros::Rate rate(50);
        while(isRobotIdle())
        {
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Pose action preempted.");
                server.setPreempted();
                return;
            }
            rate.sleep();
        }

        // Check for preemption
        while (!isRobotIdle())
        {
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Pose action preempted.");
                server.setPreempted();
                return;
            }

            rate.sleep();
        }

        // Publish result
        quadruped_control::SolveFKPoseResponse fkResponse = getCurrentPose(this->currentState.position);
        if (fkResponse.result == 0)
        {
            this->actionResult.finalPose = fkResponse.solution;
            this->actionResult.error = calculateTaskError(fkResponse);
            this->actionResult.time = ros::Time::now().toSec() - start;
            if (this->actionResult.error < this->eps)
            {
                this->actionResult.errorCode = 0;
            }
            else
            {
                this->actionResult.errorCode = -2;
            }
        }
        else
        {
            this->actionResult.errorCode = -3;
        }
        server.setSucceeded(this->actionResult);
    }

    void publishFeedback(const quadruped_control::SetJointFeedback::ConstPtr& jointFeedback)
    {
        // Get current pose
        quadruped_control::SolveFKPoseResponse fkResponse = getCurrentPose(jointFeedback->positions);
        if (fkResponse.result == 0)
        {
            this->actionFeedback.currentPose = fkResponse.solution;
            this->actionFeedback.error = calculateTaskError(fkResponse);
            this->actionFeedback.time = jointFeedback->time;
            server.publishFeedback(this->actionFeedback);
        }
    }

    void publishResult(const actionlib::SimpleClientGoalState& state, 
        const quadruped_control::SetJointResult::ConstPtr& jointResult)
    {
        
    }

    void activeCB()
    {

    }

    quadruped_control::SolveFKPoseResponse getCurrentPose(std::vector<double> config)
    {
        // Send FK request to service
        quadruped_control::SolveFKPose fkMsg;
        fkMsg.request.jointPositions = config;
        fkClient.call(fkMsg);

        return fkMsg.response;

    }

    double calculateJointError()
    {
        double yawError = this->yawTarget - currentState.position[0];
        double pitchError = this->pitchTarget - currentState.position[1];
        return sqrt(pow(yawError, 2) + pow(pitchError, 2));
    }

    double calculateTaskError(quadruped_control::SolveFKPoseResponse fkResponse)
    {
        double dx = this->targetx - fkResponse.solution.x;
        double dy = this->targety - fkResponse.solution.y;
        double dz = this->targetz - fkResponse.solution.z;
        return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
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

        return magnitude < 0.01 && jointError < 0.1;
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
    actionlib::SimpleActionServer<quadruped_control::SetPoseAction>  server;
    actionlib::SimpleActionClient<quadruped_control::SetJointAction> client;
    quadruped_control::SetPoseFeedback actionFeedback;
    quadruped_control::SetPoseResult actionResult;
    ros::ServiceClient ikClient;
    ros::ServiceClient fkClient;
    ros::Subscriber jointStateSubscriber;
    sensor_msgs::JointState currentState;
    sensor_msgs::JointState temp;
    double yawTarget;
    double pitchTarget;
    double targetx;
    double targety;
    double targetz;
    double eps;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "tail_pose_action");
    ROS_INFO("Initialized ros...");

    SetPoseAction actionServer("tail_pose_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
