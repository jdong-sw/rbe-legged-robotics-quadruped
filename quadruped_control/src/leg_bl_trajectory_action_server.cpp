#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "quadruped_control/SetPoseAction.h"
#include "quadruped_control/GaitAction.h"
#include "quadruped_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "math.h"

class SetTrajectoryAction
{
public:
    SetTrajectoryAction(std::string name):
        server(node, name, boost::bind(&SetTrajectoryAction::executeCB, this, _1), false),
        client("leg_bl_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

		ROS_INFO("Subscribing to Gait Controller...");
		this->dutyFactorSubscriber = node.subscribe("/quadruped/gait/duty_factor", 10, &SetTrajectoryAction::dutyFactorCB, this);
		this->bodyVelocitySubscriber = node.subscribe("/quadruped/gait/body_velocity", 10, &SetTrajectoryAction::bodyVelocityCB, this);
        this->stopCommandSubscriber = node.subscribe("/quadruped/gait/stop", 10, &SetTrajectoryAction::stopCommandCB, this);

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
		double start = ros::Time::now().toSec();

		this->initialPhase = goal->initialPhase;
		this->strideTime = goal->strideTime;
		this->strideHeight = goal->strideHeight;
		double eps = 0.05;
		
		int state = 0;
        bool preempted = false;
        stop = false;
        initialized = false;

		double t, elapsed;
		steps = 0;
		currentPhase = this->initialPhase;
		quadruped_control::Pose targetPose;

		ros::Rate rate(50);
		while (true)
		{
            // Check if preempted or canceled
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Trajectory action preempted, ending trajectory...");
                preempted = true;
            }

            elapsed = ros::Time::now().toSec() - start;
			t = elapsed + this->initialPhase * strideTime; // account for initialPhase

            // Calculate the current phase
			currentPhase = fmod(t / this->strideTime, 1.0);
            if (!initialized && currentPhase >= dutyFactor)
            {
                initialized = true;
            }

            // Calculate the target position
            geometry_msgs::Point position;
            if (!initialized)
            {
                position = InitialTrajectory(elapsed);
            }
            else
            {
                position = SineTrajectory(currentPhase);
            }

            if ((stop || preempted) && stage.compare("Support Phase") == 0)
            {
                break;
            }

			// Build message
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

            if (currentPhase == initialPhase)
            {
                steps++;
            }

			rate.sleep();
		}

		// Publish result
        this->actionResult.stepsTaken = steps;
        if (preempted)
        {
            server.setPreempted(this->actionResult);
        }
        else
        {
		    server.setSucceeded(this->actionResult);
        }
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

    void stopCommandCB(const std_msgs::BoolConstPtr& msg)
	{
		this->stop = msg->data;
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
    ros::Subscriber stopCommandSubscriber;
	quadruped_control::Pose currentPose;
	double initialPhase;
	double currentPhase;
	double strideTime;
	double strideHeight;
	double dutyFactor;
	double bodyVelocity;
    double xOffset = -0.1695;
    double yOffset = 0.1524;
    double zOffset = -0.2855;
    // double xOffset = -0.2;
    // double yOffset = 0.1524;
    // double zOffset = -0.275;
    int steps = 0;
    bool stop = false;
    bool initialized = false;
    std::string stage;

    geometry_msgs::Point InitialTrajectory(double elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        // Just move forward until reach initial phase
        x = xOffset - bodyVelocity * elapsed;
        y = yOffset;
        z = zOffset;
        stage = "Initialization";

        position.x = x;
        position.y = y;
        position.z = z;

        return position;
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
            x = x = xOffset + (strideLength / 2 - strideLength * supportPhase);
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
    ros::init(argc, argv, "leg_bl_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction actionServer("leg_bl_trajectory_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
