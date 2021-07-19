#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "arm_control/SetPoseAction.h"
#include "arm_control/SetJointAction.h"
#include "arm_control/SolveFKPose.h"
#include "arm_control/SolveIKPose.h"
#include "arm_control/Pose.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class SetPoseAction
{
public:
    SetPoseAction(std::string name) :
        server(node, name, boost::bind(&SetPoseAction::executeCB, this, _1), false),
        client("joint_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Joint State Server...");
        this->client.waitForServer(ros::Duration(30));

        ROS_INFO("Subscribing to Joint States...");
        this->jointStateSubscriber = node.subscribe("/arm/joint_states", 10, &SetPoseAction::jointStatesCB, this);

        ROS_INFO("Subscribing to IKPoseSolver service...");
        this->ikClient = node.serviceClient<arm_control::SolveIKPose>("/arm/ik/position");

        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<arm_control::SolveFKPose>("/arm/fk/pose");

        ROS_INFO("Starting...");
        server.start();
    }

    ~SetPoseAction()
    {
        this->node.shutdown();
    }

    void executeCB(const arm_control::SetPoseGoalConstPtr &goal)
    {
        // Run IK to get joint positions
        arm_control::SolveIKPose ikMsg;
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
        this->target1 = ikMsg.response.solution[0];
        this->target2 = ikMsg.response.solution[1];
        this->target3 = ikMsg.response.solution[2];
        this->eps = goal->eps;

        // Send joint positions to joint action client
        arm_control::SetJointGoal jointAction;
        jointAction.goal = ikMsg.response.solution;
        jointAction.eps = goal->eps;
        this->client.sendGoal(jointAction,
            boost::bind(&SetPoseAction::publishResult, this, _1, _2),
            boost::bind(&SetPoseAction::activeCB, this),
            boost::bind(&SetPoseAction::publishFeedback, this, _1));

        // Get current time
        double start = ros::Time::now().toSec();

        // Wait for joints to start moving
        ros::Rate rate(20);
        while(isRobotIdle())
        {
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
        arm_control::SolveFKPoseResponse fkResponse = getCurrentPose(this->currentState.position);
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

    void publishFeedback(const arm_control::SetJointFeedback::ConstPtr& jointFeedback)
    {
        // Get current pose
        arm_control::SolveFKPoseResponse fkResponse = getCurrentPose(jointFeedback->positions);
        if (fkResponse.result == 0)
        {
            this->actionFeedback.currentPose = fkResponse.solution;
            this->actionFeedback.error = calculateTaskError(fkResponse);
            this->actionFeedback.time = jointFeedback->time;
            server.publishFeedback(this->actionFeedback);
        }
    }

    void publishResult(const actionlib::SimpleClientGoalState& state, 
        const arm_control::SetJointResult::ConstPtr& jointResult)
    {
        
    }

    void activeCB()
    {

    }

    arm_control::SolveFKPoseResponse getCurrentPose(std::vector<double> config)
    {
        // Send FK request to service
        arm_control::SolveFKPose fkMsg;
        fkMsg.request.jointPositions = config;
        fkClient.call(fkMsg);

        return fkMsg.response;

    }

    double calculateJointError()
    {
        double error1 = this->target1 - currentState.position[0];
        double error2 = this->target2 - currentState.position[1];
        double error3 = this->target3 - currentState.position[2];
        return sqrt(pow(error1, 2) + pow(error2, 2) + pow(error3, 2));
    }

    double calculateTaskError(arm_control::SolveFKPoseResponse fkResponse)
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
        this->currentState = *msg.get();
    }

private:
    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<arm_control::SetPoseAction>  server;
    actionlib::SimpleActionClient<arm_control::SetJointAction> client;
    arm_control::SetPoseFeedback actionFeedback;
    arm_control::SetPoseResult actionResult;
    ros::ServiceClient ikClient;
    ros::ServiceClient fkClient;
    ros::Subscriber jointStateSubscriber;
    sensor_msgs::JointState currentState;
    double target1;
    double target2;
    double target3;
    double targetx;
    double targety;
    double targetz;
    double eps;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "pose_action");
    ROS_INFO("Initialized ros...");

    SetPoseAction actionServer("pose_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
