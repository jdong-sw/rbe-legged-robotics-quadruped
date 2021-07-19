#include "actionlib/server/simple_action_server.h"
#include "arm_control/SetJointAction.h"
#include "arm_control/SolveFKPose.h"
#include "arm_control/SolveIKPose.h"
#include "arm_control/Pose.h"
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
        this->jointStateSubscriber = node.subscribe("/arm/joint_states", 10, &SetJointAction::jointStatesCB, this);

        ROS_INFO("Publishing to Joint Controllers");
        this->joint1Publisher = node.advertise<std_msgs::Float64>("/quadruped/fr/hip_position_controller/command", 1);
        this->joint2Publisher = node.advertise<std_msgs::Float64>("/arm/joint2_position_controller/command", 1);
        this->joint3Publisher = node.advertise<std_msgs::Float64>("/arm/joint3_position_controller/command", 1);

        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<arm_control::SolveFKPose>("/arm/fk/pose");

        ROS_INFO("Starting...");
        server.start();
    }

    ~SetJointAction()
    {
        this->node.shutdown();
    }

    void executeCB(const arm_control::SetJointGoalConstPtr &goal)
    {
        // Set goal
        this->target1 = goal->goal[0];
        this->target2 = goal->goal[1];
        this->target3 = goal->goal[2];
        this->eps = goal->eps;

        // Command joints
        std_msgs::Float64 joint1Command, joint2Command, joint3Command;
        joint1Command.data = this->target1;
        joint2Command.data = this->target2;
        joint3Command.data = this->target3;
        joint1Publisher.publish(joint1Command);
        joint2Publisher.publish(joint2Command);
        joint3Publisher.publish(joint3Command);

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

    arm_control::SolveFKPoseResponse getCurrentPose()
    {
        // Send FK request to service
        arm_control::SolveFKPose fkMsg;
        fkMsg.request.jointPositions = this->currentState.position;
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
        this->currentState = *msg.get();
    }

private:
    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<arm_control::SetJointAction>  server;
    arm_control::SetJointFeedback actionFeedback;
    arm_control::SetJointResult actionResult;
    ros::ServiceClient fkClient;
    ros::Subscriber jointStateSubscriber;
    ros::Publisher joint1Publisher;
    ros::Publisher joint2Publisher;
    ros::Publisher joint3Publisher;
    sensor_msgs::JointState currentState;
    double target1;
    double target2;
    double target3;
    double eps;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Joint Action Server...");
    ros::init(argc, argv, "joint_action");

    SetJointAction actionServer("joint_action");
    ros::spin();
    return 0;
}
