#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "quadruped_control/SetPoseAction.h"
#include "quadruped_control/GaitAction.h"
#include "quadruped_control/Pose.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "math.h"

class SetTrajectoryAction
{
public:
    SetTrajectoryAction(std::string name):
        server(node, name, boost::bind(&SetTrajectoryAction::executeCB, this, _1), false),
        client("leg_fr_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

		ROS_INFO("Subscribing to Gait Controller...");
		this->dutyFactorSubscriber = node.subscribe("/quadruped/gait/duty_factor", 10, &SetTrajectoryAction::dutyFactorCB, this);
		this->bodyVelocitySubscriber = node.subscribe("/quadruped/gait/body_velocity", 10, &SetTrajectoryAction::bodyVelocityCB, this);

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
		
		ROS_INFO("Starting...");
		server.start();
    }

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

	void executeCB(const quadruped_control::GaitGoalConstPtr &goal)
	{
        // Get initial foot position w.r.t. body
        // auto initialFootPosition = GetFootPosition();
        // xOffset = initialFootPosition.x;
        // yOffset = initialFootPosition.y;
        // zOffset = initialFootPosition.z;

		double start = ros::Time::now().toSec();

		this->initialPhase = goal->initialPhase;
		this->strideTime = goal->strideTime;
		this->strideHeight = goal->strideHeight;
		double eps = 0.05;
		
		int state = 0;

		double t;
		int stepsTaken = 0;
		currentPhase = this->initialPhase;
		quadruped_control::Pose targetPose;

		ros::Rate rate(50);
		while (true)
		{
			t = ros::Time::now().toSec() - start + this->initialPhase * strideTime; // account for initialPhase

			currentPhase = fmod(t / this->strideTime, 1.0);
			auto position = SineTrajectory(currentPhase);

			// Calculate the targetPose
			targetPose.x = position.x;
			targetPose.y = position.y;
			targetPose.z = position.z;
			targetPose.rotx = std::vector<double>{1.0, 0.0, 0.0};
			targetPose.roty = std::vector<double>{0.0, 1.0, 0.0};
			targetPose.rotz = std::vector<double>{0.0, 0.0, 1.0};

			// Send parameters to pose action client
			quadruped_control::SetPoseGoal poseAction;
			poseAction.goal = targetPose;
			poseAction.eps = eps;
			this->client.sendGoal(poseAction,
				boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
				boost::bind(&SetTrajectoryAction::activeCB, this),
				boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));

			this->actionFeedback.currentPhase = currentPhase;
			this->actionFeedback.targetPose = targetPose;
			this->actionFeedback.currentPose = currentPose;
            this->actionFeedback.stage = stage;
			server.publishFeedback(this->actionFeedback);

			rate.sleep();
		}

		// Publish result
		this->actionResult.stepsTaken = stepsTaken;
		server.setSucceeded(this->actionResult);
	}

	void publishFeedback(const quadruped_control::SetPoseFeedback::ConstPtr& poseFeedback)
	{
		currentPose = poseFeedback->currentPose;
	}

	void publishResult(const actionlib::SimpleClientGoalState& state,
		const quadruped_control::SetPoseResult::ConstPtr& poseResult)
	{

	}

	void activeCB()
	{

	}

	void dutyFactorCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->dutyFactor = msg->data;
	}

	void bodyVelocityCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->bodyVelocity = msg->data;
	}

private:
	std::string actionName;
	ros::NodeHandle node;
	actionlib::SimpleActionServer<quadruped_control::GaitAction> server;
	actionlib::SimpleActionClient<quadruped_control::SetPoseAction> client;
    ros::ServiceClient linkStateClient;
	quadruped_control::GaitFeedback actionFeedback;
	quadruped_control::GaitResult actionResult;
	ros::Subscriber dutyFactorSubscriber;
	ros::Subscriber bodyVelocitySubscriber;
	quadruped_control::Pose currentPose;
	double initialPhase;
	double currentPhase;
	double strideTime;
	double strideHeight;
	double dutyFactor;
	double bodyVelocity;
    // 0.1778 -0.1524 0
    // double xOffset = 0.1778;
    // double yOffset = -0.1524;
    // double zOffset = -0.305;
    double xOffset = 0.0639;
    double yOffset = -0.1524;
    double zOffset = -0.2855;
    std::string stage;

    geometry_msgs::Point GetFootPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "foot_fr";
        linkStateMsg.request.reference_frame = "body";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    geometry_msgs::Point SineTrajectory(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        double strideLength = strideTime * bodyVelocity;

        // Support phase
        if (phase < dutyFactor)
        {
            double supportPhase = phase / dutyFactor;
            x = xOffset + (strideLength / 2 - strideLength * supportPhase);
            y = yOffset;
            z = zOffset;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            double transferPhase = (phase - dutyFactor) / (1.0 - dutyFactor);
            x = xOffset + (strideLength * transferPhase - strideLength / 2);
            y = yOffset;
            z = 0.5 * strideHeight * cos(2 * M_PI * transferPhase - M_PI) + zOffset + strideHeight/2;
            stage = "Transfer Phase";
        }
        
        position.x = x;
        position.y = y;
        position.z = z;

        return position;
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Trajectory Action Server...");
    ros::init(argc, argv, "leg_fr_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction actionServer("leg_fr_trajectory_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
