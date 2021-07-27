#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "quadruped_control/GaitAction.h"
#include "quadruped_control/MoveAction.h"
#include "quadruped_control/Pose.h"
#include "quadruped_control/VectorOfPoints.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
// #include <tf2/impl/convert.h>
// #include <tf2/transform_datatypes.h>
// #include <Transform.h>

namespace tf2
{
    //overload "convert" to convert from geometry_msgs::TransformStamped to tf2::Transform
    inline
    void convert(const geometry_msgs::TransformStamped& transGEO, tf2::Transform& transTF2)
    {
        transTF2.setOrigin(tf2::Vector3(transGEO.transform.translation.x, transGEO.transform.translation.y, transGEO.transform.translation.z));
        transTF2.setRotation(tf2::Quaternion(transGEO.transform.rotation.x, transGEO.transform.rotation.y, transGEO.transform.rotation.z, transGEO.transform.rotation.w));
    }
}

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
        this->dutyRatioPublisher = node.advertise<std_msgs::Float64>("/quadruped/gait/duty_factor", 1);
        this->bodyVelocityPublisher = node.advertise<std_msgs::Float64>("/quadruped/gait/body_velocity", 1);
        this->COMPublisher = node.advertise<geometry_msgs::Point>("/quadruped/gait/COM", 1);
        this->supportStructurePublisher = node.advertise<quadruped_control::VectorOfPoints>("/quadruped/gait/supportStructure", 1);

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        this->linkPropertiesClient = node.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");

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
        double bodyVelocity, bodyAcceleration, strideTime, strideHeight;
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
        auto initialPosition = GetPosition("body");

        // Get current time
        double start = ros::Time::now().toSec();

        // Publish initial gait parameters
        std_msgs::Float64 msg;
        msg.data = dutyRatio;
        this->dutyRatioPublisher.publish(msg);

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
        double distanceTraveled, velocity, stride, rampTime, rampDistance;
        bool preempted = false;
        bool aborted = false;
        std::string description = "Ramping up.";
        
        while (true)
        {
            // Calculate elapsed time
            double elapsed = ros::Time::now().toSec() - start;

            // Calculate distance traveled
            auto currentPosition = GetPosition("body");
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
                }
            }
            else if (stage == 1) // Constant velocity stage
            {
                if (distanceTraveled >= targetDistance - rampDistance)
                {
                    stage = 2;
                    description = "Slowing down.";
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
                    msg.data = 1;
                    this->dutyRatioPublisher.publish(msg);
                    stage = 3;
                    description = "Stopping.";
                }
            }
            else // Stopping stage
            {
                if (!frIsActive && !flIsActive && !brIsActive && !blIsActive)
                {
                    break;
                }
            }
            
            // Publish feedback
            this->actionFeedback.distance = distanceTraveled;
            this->actionFeedback.time = elapsed;
            this->actionFeedback.description = description;
            server.publishFeedback(this->actionFeedback);

            ROS_INFO("COM: (%f, %f, %f)", COM_pos.x, COM_pos.y, COM_pos.z);

            double stabilityMargin = CalcStabilityMargin();
            ROS_INFO("SM: %f", stabilityMargin);

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
        frCurrentPhase = gaitFeedback->currentPhase;
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
        flCurrentPhase = gaitFeedback->currentPhase;
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
        brCurrentPhase = gaitFeedback->currentPhase;
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
        blCurrentPhase = gaitFeedback->currentPhase;
    }

    void blResult(const actionlib::SimpleClientGoalState &state,
                  const quadruped_control::GaitResult::ConstPtr &gaitResult)
    {
        blIsActive = false;
    }

private:

    geometry_msgs::Point GetPosition(std::string linkName)
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = linkName;
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    tf2::Vector3 GetLinkCOM(std::string linkName)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform(linkName, "body", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // tf2::Transform linkToCOMTransform;
        // linkToCOMTransform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0));
        geometry_msgs::TransformStamped linkToCOMTransform;
        linkToCOMTransform.header.frame_id = linkName.substr(0, linkName.size()-4);
        linkToCOMTransform.child_frame_id = linkName;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        linkToCOMTransform.transform.rotation.x = q.x();
        linkToCOMTransform.transform.rotation.y = q.y();
        linkToCOMTransform.transform.rotation.z = q.z();
        linkToCOMTransform.transform.rotation.w = q.w();
        if (linkName == "body_com")
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.01781, 0.0, 0.00882));
            linkToCOMTransform.transform.translation.x = 0.01781;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.00882;
        }
        else if (linkName == "tailbase_com")
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.04244));
            linkToCOMTransform.transform.translation.x = 0.0;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.04244;
        }
        else if (linkName == "tail_com")
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.15694, 0.0, 0.0));
            linkToCOMTransform.transform.translation.x = 0.15694;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.0;
        }
        else if (linkName.find("coxa") != std::string::npos)
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.04183));
            linkToCOMTransform.transform.translation.x = 0.0;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.04183;
        }
        else if (linkName.find("femur") != std::string::npos)
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.08660, 0.0, 0.0));
            linkToCOMTransform.transform.translation.x = 0.08660;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.0;
        }
        else if (linkName.find("tibia") != std::string::npos)
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.10609));
            linkToCOMTransform.transform.translation.x = 0.0;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.10609;
        }

        tf2::Transform bodyToLinkTF2Transform, bodyToCOMTF2Transform, linkToCOMTF2Transform;
        tf2::convert(transformStamped, bodyToLinkTF2Transform);
        tf2::convert(linkToCOMTransform, linkToCOMTF2Transform);
        // tf2::fromMsg(transformStamped, bodyToLinkTF2Transform);
        // tf2::fromMsg(linkToCOMTransform, linkToCOMTF2Transform);
        bodyToCOMTF2Transform.mult(bodyToLinkTF2Transform, linkToCOMTF2Transform);
        return bodyToCOMTF2Transform.getOrigin();
    }

    double GetMass(std::string linkName)
    {
        gazebo_msgs::GetLinkProperties linkPropertiesMsg;
        linkPropertiesMsg.request.link_name = linkName;
        this->linkPropertiesClient.call(linkPropertiesMsg);
        return linkPropertiesMsg.response.mass;
    }

    geometry_msgs::Point GetCOM()
    {
        geometry_msgs::Point COM;
        double COM_x, COM_y, COM_z;
        tf2::Vector3 body_com_pos,    tailbase_com_pos, tail_com_pos;
        tf2::Vector3 coxa_fr_com_pos, femur_fr_com_pos, tibia_fr_com_pos;
        tf2::Vector3 coxa_fl_com_pos, femur_fl_com_pos, tibia_fl_com_pos;
        tf2::Vector3 coxa_br_com_pos, femur_br_com_pos, tibia_br_com_pos;
        tf2::Vector3 coxa_bl_com_pos, femur_bl_com_pos, tibia_bl_com_pos;

        double body_mass,    tailbase_mass, tail_mass;
        double coxa_fr_mass, femur_fr_mass, tibia_fr_mass;
        double coxa_fl_mass, femur_fl_mass, tibia_fl_mass;
        double coxa_br_mass, femur_br_mass, tibia_br_mass;
        double coxa_bl_mass, femur_bl_mass, tibia_bl_mass;

        // Get COM of each robot link
        body_com_pos    = GetLinkCOM("body_com");    tailbase_com_pos = GetLinkCOM("tailbase_com"); tail_com_pos     = GetLinkCOM("tail_com");
        coxa_fr_com_pos = GetLinkCOM("coxa_fr_com"); femur_fr_com_pos = GetLinkCOM("femur_fr_com"); tibia_fr_com_pos = GetLinkCOM("tibia_fr_com");
        coxa_fl_com_pos = GetLinkCOM("coxa_fl_com"); femur_fl_com_pos = GetLinkCOM("femur_fl_com"); tibia_fl_com_pos = GetLinkCOM("tibia_fl_com");
        coxa_br_com_pos = GetLinkCOM("coxa_br_com"); femur_br_com_pos = GetLinkCOM("femur_br_com"); tibia_br_com_pos = GetLinkCOM("tibia_br_com");
        coxa_bl_com_pos = GetLinkCOM("coxa_bl_com"); femur_bl_com_pos = GetLinkCOM("femur_bl_com"); tibia_bl_com_pos = GetLinkCOM("tibia_bl_com");

        // Get mass of each robot link (can also hard-code these)
        body_mass    = GetMass("body");    tailbase_mass = GetMass("tailbase"); tail_mass     = GetMass("tail");
        coxa_fr_mass = GetMass("coxa_fr"); femur_fr_mass = GetMass("femur_fr"); tibia_fr_mass = GetMass("tibia_fr");
        coxa_fl_mass = GetMass("coxa_fl"); femur_fl_mass = GetMass("femur_fl"); tibia_fl_mass = GetMass("tibia_fl");
        coxa_br_mass = GetMass("coxa_br"); femur_br_mass = GetMass("femur_br"); tibia_br_mass = GetMass("tibia_br");
        coxa_bl_mass = GetMass("coxa_bl"); femur_bl_mass = GetMass("femur_bl"); tibia_bl_mass = GetMass("tibia_bl");

        // Calculate sum of positions * masses
        COM_x = body_mass*body_com_pos.x()       + tailbase_mass*tailbase_com_pos.x() + tail_mass*tail_com_pos.x() + 
                coxa_fr_mass*coxa_fr_com_pos.x() + femur_fr_mass*femur_fr_com_pos.x() + tibia_fr_mass*tibia_fr_com_pos.x() + 
                coxa_fl_mass*coxa_fl_com_pos.x() + femur_fl_mass*femur_fl_com_pos.x() + tibia_fl_mass*tibia_fl_com_pos.x() + 
                coxa_br_mass*coxa_br_com_pos.x() + femur_br_mass*femur_br_com_pos.x() + tibia_br_mass*tibia_br_com_pos.x() + 
                coxa_bl_mass*coxa_bl_com_pos.x() + femur_bl_mass*femur_bl_com_pos.x() + tibia_bl_mass*tibia_bl_com_pos.x();
        COM_y = body_mass*body_com_pos.y()       + tailbase_mass*tailbase_com_pos.y() + tail_mass*tail_com_pos.y() + 
                coxa_fr_mass*coxa_fr_com_pos.y() + femur_fr_mass*femur_fr_com_pos.y() + tibia_fr_mass*tibia_fr_com_pos.y() + 
                coxa_fl_mass*coxa_fl_com_pos.y() + femur_fl_mass*femur_fl_com_pos.y() + tibia_fl_mass*tibia_fl_com_pos.y() + 
                coxa_br_mass*coxa_br_com_pos.y() + femur_br_mass*femur_br_com_pos.y() + tibia_br_mass*tibia_br_com_pos.y() + 
                coxa_bl_mass*coxa_bl_com_pos.y() + femur_bl_mass*femur_bl_com_pos.y() + tibia_bl_mass*tibia_bl_com_pos.y();
        COM_z = body_mass*body_com_pos.z()       + tailbase_mass*tailbase_com_pos.z() + tail_mass*tail_com_pos.z() + 
                coxa_fr_mass*coxa_fr_com_pos.z() + femur_fr_mass*femur_fr_com_pos.z() + tibia_fr_mass*tibia_fr_com_pos.z() + 
                coxa_fl_mass*coxa_fl_com_pos.z() + femur_fl_mass*femur_fl_com_pos.z() + tibia_fl_mass*tibia_fl_com_pos.z() + 
                coxa_br_mass*coxa_br_com_pos.z() + femur_br_mass*femur_br_com_pos.z() + tibia_br_mass*tibia_br_com_pos.z() + 
                coxa_bl_mass*coxa_bl_com_pos.z() + femur_bl_mass*femur_bl_com_pos.z() + tibia_bl_mass*tibia_bl_com_pos.z();
        COM.x = COM_x;
        COM.y = COM_y;
        COM.z = COM_z;  
        return COM;
    }

    static bool sortByXCoord(geometry_msgs::Point point1, geometry_msgs::Point point2)
    {
        return (point1.x < point2.x);
    }

    double CalcStabilityMargin()
    {
        geometry_msgs::Point foot_fr_pos, foot_fl_pos, foot_br_pos, foot_bl_pos;
        foot_fr_pos = GetPosition("foot_fr");
        foot_fl_pos = GetPosition("foot_fl");
        foot_br_pos = GetPosition("foot_br");
        foot_bl_pos = GetPosition("foot_bl");

        // find which legs are on the ground and create list of indices representing support structure (triangle or parallelogram)
        quadruped_control::VectorOfPoints supportStructure;
        if (frCurrentPhase < dutyRatio)
            supportStructure.points.push_back(foot_fr_pos);
        if (flCurrentPhase < dutyRatio)
            supportStructure.points.push_back(foot_fl_pos);
        if (brCurrentPhase < dutyRatio)
            supportStructure.points.push_back(foot_br_pos);
        if (blCurrentPhase < dutyRatio)
            supportStructure.points.push_back(foot_bl_pos);

        // sort on the x coordinate in ascending order
        sort(supportStructure.points.begin(), supportStructure.points.end(), sortByXCoord);
        
        geometry_msgs::Point pointMax1 = supportStructure.points.end()[-1]; // last element
        geometry_msgs::Point pointMax2 = supportStructure.points.end()[-2]; // second to last element
        geometry_msgs::Point pointMin1 = supportStructure.points[0]; // first element
        geometry_msgs::Point pointMin2 = supportStructure.points[1]; // second element
        COM_pos = GetCOM();

        // Publish values to tail controller
        this->COMPublisher.publish(COM_pos);
        this->supportStructurePublisher.publish(supportStructure);

        // Find distance from COM_x to edge formed by two max points in x direction
        double distance1 = distancePointToEdge(COM_pos, pointMax1, pointMax2);

        // Find distance from COM_x to edge formed by two min points in x direction
        double distance2 = distancePointToEdge(COM_pos, pointMin1, pointMin2);

        // Find minimum between two distances calculated, normalize by dividing by other distance
        double stabilityMargin;
        if (distance1 < distance2)
            stabilityMargin = distance1 / distance2;
        else
            stabilityMargin = distance2 / distance1;

        return stabilityMargin;
    }

    double distancePointToEdge(geometry_msgs::Point COM_pos, geometry_msgs::Point point1, geometry_msgs::Point point2)
    {
        // solve for x distance only between COM and horizontal x intersection with line between point 1 and point 2
        return (COM_pos.y - point1.y)*(point2.x - point1.x)/(point2.y - point1.y) + point1.x - COM_pos.x;
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
    double dutyRatio;
    double frCurrentPhase;
    double flCurrentPhase;
    double brCurrentPhase;
    double blCurrentPhase;
    geometry_msgs::Point COM_pos;
    quadruped_control::MoveFeedback actionFeedback;
    quadruped_control::MoveResult actionResult;
    ros::Publisher dutyRatioPublisher;
    ros::Publisher bodyVelocityPublisher;
    ros::Publisher COMPublisher;
    ros::Publisher supportStructurePublisher;
    ros::ServiceClient linkStateClient;
    ros::ServiceClient linkPropertiesClient;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Gait Action Server...");
    ros::init(argc, argv, "gait_controller_node");
    ROS_INFO("Initialized ros...");

    GaitController actionServer("gait_controller");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
