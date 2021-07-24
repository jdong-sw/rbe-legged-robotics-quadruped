#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "quadruped_control/SetPoseAction.h"
#include "quadruped_control/GaitAction.h"
#include "quadruped_control/Pose.h"
#include "std_msgs/Float64.h"
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
		
		ROS_INFO("Starting...");
		server.start();
    }

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

	void executeCB(const quadruped_control::GaitGoalConstPtr &goal)
	{
		double start = ros::Time::now().toSec();

		this->initialPhase = goal->initialPhase;
		this->strideTime = goal->strideTime;
		this->strideHeight = goal->strideHeight;
		double eps = 0.05;

		double xOffset = 0.1778; // from urdf
		double yOffset = -0.1524; // from urdf
		double zOffset = -0.305; // from Gazebo with default leg angles
		
		int state = 0;
		double x = xOffset;
		double y = yOffset;
		double z = zOffset;

		double t, last_t;
		int stepsTaken = 0;
		currentPhase = this->initialPhase;
		quadruped_control::Pose targetPose;

		ros::Rate rate(50);
		while (true)
		{
			double strideLength = this->strideTime * this->bodyVelocity;
			double omega = M_PI / this->strideTime;
			
			// Solve for cubic coefficients of x(t) = a*t^3 + b*t^2 + c*t + d
			// using 4 equations x(0) = 0, x(T) = L, dx(0) = 0, dx(T) = 0. c and d are 0
			double a = -2 * strideLength / pow(this->strideTime, 3);
			double b = 3 * strideLength / pow(this->strideTime, 2);

			t = ros::Time::now().toSec() - start + this->initialPhase * strideTime; // account for initialPhase

			currentPhase = fmod(t / this->strideTime, 1.0);
			if (currentPhase >= this->dutyFactor) // foot in air
			{
				state = 1;
				x = a * pow(t, 3) + b * pow(t, 2) + xOffset; // position of foot wrt ground
				x -= bodyVelocity * (t - last_t); // position of foot wrt body
				y = yOffset;
				z = strideHeight * sin(omega * currentPhase * this->strideTime) + zOffset;
			}
			else // foot on ground
			{
				if (state == 1)
				{
					stepsTaken += 1; // increment only if changing states
				}
				if (this->dutyFactor == 1.0)
				{
					break;
				}
				state = 0;
				x -= bodyVelocity * (t - last_t);
				y = yOffset;
				z = zOffset;
			}

			// Calculate the targetPose
			targetPose.x = x;
			targetPose.y = y;
			targetPose.z = z;
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
			this->actionFeedback.currentPose = targetPose;
			// this->actionFeedback.currentPose = currentPose;
			server.publishFeedback(this->actionFeedback);

			last_t = t;

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
