#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "quadruped_control/GaitAction.h"
#include "quadruped_control/MoveAction.h"
#include "quadruped_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/GetLinkState.h"

class GaitController
{
public:
    GaitController(std::string name) : server(node, name, boost::bind(&GaitController::executeCB, this, _1), false),
                                       fr_client("leg_fr_trajectory_action", true),
                                       fl_client("leg_fl_trajectory_action", true),
                                       br_client("leg_br_trajectory_action", true),
                                       bl_client("leg_bl_trajectory_action", true),
                                       actionName(name)
    {
        this->node = node;

        ROS_INFO("Starting gait parameters publishers...");
        this->dutyRatioPublisher = node.advertise<std_msgs::Float64>(
            "/quadruped/gait/duty_factor", 1);
        this->bodyVelocityPublisher = node.advertise<std_msgs::Float64>(
            "/quadruped/gait/body_velocity", 1);
        this->stopCommandPublisher = node.advertise<std_msgs::Bool>(
            "/quadruped/gait/stop", 1);

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Waiting for Leg Trajectory Servers...");
        this->fr_client.waitForServer(ros::Duration(30));
        this->fl_client.waitForServer(ros::Duration(30));
        this->br_client.waitForServer(ros::Duration(30));
        this->bl_client.waitForServer(ros::Duration(30));

        ROS_INFO("Starting Gait Action Server...");
        server.start();

        ROS_INFO("Gait controller ready.");
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

    void executeCB(const quadruped_control::MoveGoalConstPtr &goal)
    {
        // Extract goal
        int gaitType = goal->gaitType;
        double targetDistance = goal->distance;
        double targetTime = goal->time;
        double initialPhase[4];
        ROS_INFO("Gait Controller goal received.");

        // Calculate gait parameters
        int numSteps;
        double bodyVelocity, bodyAcceleration, strideTime, strideHeight, dutyRatio;
        std::vector<double> relative_phases;
        if (gaitType == 0) // Walking
        {
            // Get parameters from parameter server
            node.getParam("/quadruped/gait/walking/velocity", bodyVelocity);
            node.getParam("/quadruped/gait/walking/acceleration", bodyAcceleration);
            node.getParam("/quadruped/gait/walking/duty_factor", dutyRatio);
            node.getParam("/quadruped/gait/walking/relative_phase", relative_phases);
            node.getParam("/quadruped/gait/walking/stride_time", strideTime);
            node.getParam("/quadruped/gait/walking/stride_height", strideHeight);
        }

        // Get initial position
        auto initialPosition = GetPosition();

        // Get current time
        double start = ros::Time::now().toSec();

        // Publish initial gait parameters
        std_msgs::Float64 msg;
        msg.data = dutyRatio;
        this->dutyRatioPublisher.publish(msg);
        
        std_msgs::Bool stopCommand;
        stopCommand.data = true;

        double velocity = 0.0;
        msg.data = velocity;
        this->bodyVelocityPublisher.publish(msg);

        // Send goals to trajectory servers
        ROS_INFO("Initializing leg trajectories...");
        quadruped_control::GaitGoal gaitAction;
        gaitAction.strideTime = strideTime;
        gaitAction.strideHeight = strideHeight;
        gaitAction.initialPhase = relative_phases[0];
        this->fl_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::flResult, this, _1, _2),
                                 boost::bind(&GaitController::flActive, this),
                                 boost::bind(&GaitController::flFeedback, this, _1));
        gaitAction.initialPhase = relative_phases[1];
        this->fr_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::frResult, this, _1, _2),
                                 boost::bind(&GaitController::frActive, this),
                                 boost::bind(&GaitController::frFeedback, this, _1));
        gaitAction.initialPhase = relative_phases[2];
        this->bl_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::blResult, this, _1, _2),
                                 boost::bind(&GaitController::blActive, this),
                                 boost::bind(&GaitController::blFeedback, this, _1));
        gaitAction.initialPhase = relative_phases[3];
        this->br_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::brResult, this, _1, _2),
                                 boost::bind(&GaitController::brActive, this),
                                 boost::bind(&GaitController::brFeedback, this, _1));

        // Start gait loop
        ROS_INFO("Starting gait...");
        ros::Rate rate(50);
        int stage = 0;
        double distanceTraveled, stride, rampTime, rampDistance;
        bool preempted = false;
        bool aborted = false;
        std::string description = "Ramping up.";

        ROS_INFO("Ramp up stage.");
        while (true)
        {
            // Calculate elapsed time
            double elapsed = ros::Time::now().toSec() - start;

            // Calculate distance traveled
            auto currentPosition = GetPosition();
            distanceTraveled = sqrt(pow(currentPosition.x - initialPosition.x, 2) + 
                                    pow(currentPosition.y - initialPosition.y, 2));

            // Check if preempted
            if (server.isPreemptRequested())
            {
                ROS_INFO("Gait action preempted, ending gait...");
                preempted = true;
                stage = 2;
                description = "Slowing down.";
            }
            else if (!ros::ok())
            {
                ROS_INFO("Gait action aborted, ending gait...");
                aborted = true;
                stage = 2;
                description = "Slowing down.";
            }

            // Control gait
            if (stage == 0) // Ramp-up stage
            {
                 // Ramping up
                if (velocity < bodyVelocity)
                {
                    velocity = velocity + elapsed*bodyAcceleration;
                    msg.data = velocity;
                    this->bodyVelocityPublisher.publish(msg);
                }
                // Hit target velocity
                else
                {
                    msg.data = bodyVelocity;
                    this->bodyVelocityPublisher.publish(msg);
                    rampTime = elapsed;
                    rampDistance = distanceTraveled;
                    stage = 1;
                    description = "Constant velocity.";
                    ROS_INFO("Constant velocity stage.");
                }
            }
            else if (stage == 1) // Constant velocity stage
            {
                msg.data = bodyVelocity;
                this->bodyVelocityPublisher.publish(msg);
                if (distanceTraveled >= targetDistance - rampDistance)
                {
                    stage = 2;
                    description = "Slowing down.";
                    ROS_INFO("Slowing down stage.");
                }
            }
            else if (stage == 2) // Slow-down stage
            {
                // Slowing down
                if (velocity > 0)
                {
                    velocity = velocity - elapsed*bodyAcceleration;
                    if (velocity < 0)
                    {
                        velocity = 0;
                    }
                    msg.data = velocity;
                    this->bodyVelocityPublisher.publish(msg);
                }
                // Hit zero velocity
                else
                {
                    this->stopCommandPublisher.publish(stopCommand);
                    stage = 3;
                    description = "Stopping.";
                    ROS_INFO("Stopping stage.");
                }
            }
            else // Stopping stage
            {
                this->stopCommandPublisher.publish(stopCommand);
                if (!frIsActive && !flIsActive && !brIsActive && !blIsActive)
                {
                    ROS_INFO("Stopped.");
                    break;
                }
            }
            
            // Publish feedback
            this->actionFeedback.distance = distanceTraveled;
            this->actionFeedback.time = elapsed;
            this->actionFeedback.description = description;
            server.publishFeedback(this->actionFeedback);

            rate.sleep();
        }

        // Publish result
        this->actionResult.distance = distanceTraveled;
        this->actionResult.time = ros::Time::now().toSec() - start;

        if (preempted)
        {
            server.setPreempted(actionResult);
        }
        else if (aborted)
        {
            server.setAborted(actionResult);
        }
        else
        {
            server.setSucceeded(actionResult);
        }
    }

    void frActive()
    {
        frIsActive = true;
    }

    void frFeedback(const quadruped_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void frResult(const actionlib::SimpleClientGoalState &state,
                  const quadruped_control::GaitResult::ConstPtr &gaitResult)
    {
        frIsActive = false;
    }

    void flActive()
    {
        flIsActive = true;
    }

    void flFeedback(const quadruped_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void flResult(const actionlib::SimpleClientGoalState &state,
                  const quadruped_control::GaitResult::ConstPtr &gaitResult)
    {
        flIsActive = false;
    }

    void brActive()
    {
        brIsActive = true;
    }

    void brFeedback(const quadruped_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void brResult(const actionlib::SimpleClientGoalState &state,
                  const quadruped_control::GaitResult::ConstPtr &gaitResult)
    {
        brIsActive = false;
    }

    void blActive()
    {
        blIsActive = true;
    }

    void blFeedback(const quadruped_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void blResult(const actionlib::SimpleClientGoalState &state,
                  const quadruped_control::GaitResult::ConstPtr &gaitResult)
    {
        blIsActive = false;
    }

private:

    geometry_msgs::Point GetPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "body";
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<quadruped_control::MoveAction> server;
    actionlib::SimpleActionClient<quadruped_control::GaitAction> fr_client;
    actionlib::SimpleActionClient<quadruped_control::GaitAction> fl_client;
    actionlib::SimpleActionClient<quadruped_control::GaitAction> br_client;
    actionlib::SimpleActionClient<quadruped_control::GaitAction> bl_client;
    bool frIsActive = false;
    bool flIsActive = false;
    bool blIsActive = false;
    bool brIsActive = false;
    quadruped_control::MoveFeedback actionFeedback;
    quadruped_control::MoveResult actionResult;
    ros::Publisher dutyRatioPublisher;
    ros::Publisher bodyVelocityPublisher;
    ros::Publisher stopCommandPublisher;
    ros::ServiceClient linkStateClient;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "gait_controller_node");
    ROS_INFO("Initialized ros...");

    GaitController actionServer("gait_controller");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
