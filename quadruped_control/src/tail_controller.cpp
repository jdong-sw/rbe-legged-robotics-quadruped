#include "actionlib/server/simple_action_server.h"
#include "quadruped_control/TailAction.h"
#include "quadruped_control/VectorOfPoints.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"

class TailController
{
public:
    TailController(std::string name) : server(node, name, boost::bind(&TailController::executeCB, this, _1), false),
                                       actionName(name)
    {
        this->node = node;

        ROS_INFO("Subscribing to COM service");
        this->COMSubscriber = node.subscribe("/quadruped/gait/COM", 10, &TailController::COMCB, this);
        this->supportStructureSubscriber = node.subscribe("/quadruped/gait/supportStructure", 10, &TailController::supportStructureCB, this);

        ROS_INFO("Publishing to Tail Controllers");
        this->tailYawPublisher = node.advertise<std_msgs::Float64>("/quadruped/tail/yaw_joint_position_controller/command", 1);
        this->tailPitchPublisher = node.advertise<std_msgs::Float64>("/quadruped/tail/pitch_joint_position_controller/command", 1);

        ROS_INFO("Starting Tail Action Server...");
        server.start();
        
        ROS_INFO("Tail controller ready.");
    }

    ~TailController()
    {
        this->node.shutdown();
    }

    void executeCB(const quadruped_control::TailGoalConstPtr &goal)
    {
        double gain = goal->gain;

        // Start tail loop
        ROS_INFO("Starting tail loop...");
        ros::Rate rate(50);
        bool preempted = false;
        bool aborted = false;

        double start = ros::Time::now().toSec();

        while (true)
        {
            // Calculate the center of the feet positions
            double total_x, total_y, total_z, average_x, average_y, average_z;
            for (int i = 0; i < supportStructure.size(); i++)
            {
                total_x += supportStructure[i].x;
                total_y += supportStructure[i].y;
                total_z += supportStructure[i].z;
            }
            average_x = total_x / supportStructure.size();
            average_y = total_y / supportStructure.size();
            average_z = total_z / supportStructure.size();

            // Calculate which direction to move the tail
            geometry_msgs::Point weightVector;
            weightVector.x = average_x - COM_pos.x;
            weightVector.y = average_y - COM_pos.y;
            weightVector.z = average_z - COM_pos.z;

            // Move the tail in the direction weightVector
            double yawAngle = atan2(weightVector.y, weightVector.x);
            double pitchAngle = gain * (weightVector.x * weightVector.x + weightVector.y * weightVector.y);
            std_msgs::Float64 tailYawCommand, tailPitchCommand;
            tailYawCommand.data = yawAngle;
            tailPitchCommand.data = pitchAngle;
            tailYawPublisher.publish(tailYawCommand);
            tailPitchPublisher.publish(tailPitchCommand);

            // Check if preempted
            if (server.isPreemptRequested())
            {
                ROS_INFO("Tail action preempted, ending tail...");
                preempted = true;
            }
            else if (!ros::ok())
            {
                ROS_INFO("Tail action aborted, ending tail...");
                aborted = true;
            }

            // Publish feedback
            this->actionFeedback.time = ros::Time::now().toSec() - start;
            server.publishFeedback(this->actionFeedback);

            rate.sleep();
        }

        // Publish result
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

    void COMCB(const geometry_msgs::PointConstPtr& msg)
    {
        this->COM_pos = *msg.get();
    }

    void supportStructureCB(const quadruped_control::VectorOfPointsConstPtr& msg)
    {
        this->supportStructure = msg->points;
    }

private:
    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<quadruped_control::TailAction> server;
    geometry_msgs::Point COM_pos;
    std::vector<geometry_msgs::Point> supportStructure;
    ros::Publisher tailYawPublisher;
    ros::Publisher tailPitchPublisher;
    quadruped_control::TailFeedback actionFeedback;
    quadruped_control::TailResult actionResult;
    ros::Subscriber COMSubscriber;
    ros::Subscriber supportStructureSubscriber;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Tail Action Server...");
    ros::init(argc, argv, "tail_controller_node");
    ROS_INFO("Initialized ros...");

    TailController actionServer("tail_controller");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
