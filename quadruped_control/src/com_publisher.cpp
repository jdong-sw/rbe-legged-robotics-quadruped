#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "quadruped_control/VectorOfPoints.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

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

class COMPublisherNode
{
public:
    COMPublisherNode(ros::NodeHandle node) : tfListener(tfBuffer)
    {
        this->node = node;

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        this->linkPropertiesClient = node.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");

        ROS_INFO("Subscribing to force sensor topics...");
        this->frSensorSubscriber = node.subscribe("/tibia_fr_force", 1, &COMPublisherNode::frSensorCB, this);
        this->flSensorSubscriber = node.subscribe("/tibia_fl_force", 1, &COMPublisherNode::flSensorCB, this);
        this->brSensorSubscriber = node.subscribe("/tibia_br_force", 1, &COMPublisherNode::brSensorCB, this);
        this->blSensorSubscriber = node.subscribe("/tibia_bl_force", 1, &COMPublisherNode::blSensorCB, this);

        ROS_INFO("Starting COM publishers...");
        this->COMPublisher = node.advertise<geometry_msgs::Point>("/quadruped/gait/COM", 1);
        this->supportStructurePublisher = node.advertise<quadruped_control::VectorOfPoints>("/quadruped/gait/supportStructure", 1);
        this->stabilityMarginPublisher = node.advertise<std_msgs::Float64>("/stability/margin", 1);

        ROS_INFO("Publishing stability...");
        timer = node.createTimer(ros::Duration(1.0 / 10.0),
                        std::bind(&COMPublisherNode::PublishStability, this));

        ROS_INFO("COM publisher ready.");
    }

    ~COMPublisherNode()
    {
        this->node.shutdown();
    }


private:
    void frSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        frForce = msg->data;
    }

    void flSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        flForce = msg->data;
    }

    void brSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        brForce = msg->data;
    }

    void blSensorCB(const std_msgs::Float64ConstPtr& msg)
    {
        blForce = msg->data;
    }

    tf2::Transform poseToTransform(geometry_msgs::Pose pose)
    {
        tf2::Transform transform;
        tf2::Vector3 position;
        tf2::Quaternion rotation;

        position.setX(pose.position.x);
        position.setY(pose.position.y);
        position.setZ(pose.position.z);

        rotation.setX(pose.orientation.x);
        rotation.setY(pose.orientation.y);
        rotation.setZ(pose.orientation.z);
        rotation.setW(pose.orientation.w);

        transform.setOrigin(position);
        transform.setRotation(rotation);

        return transform;
    }

    tf2::Transform vectorToTransform(geometry_msgs::Vector3 vector)
    {
        tf2::Transform transform;
        tf2::Vector3 position;
        tf2::Quaternion rotation;

        position.setX(vector.x);
        position.setY(vector.y);
        position.setZ(vector.z);
        rotation.setRPY(0, 0, 0);

        transform.setOrigin(position);
        transform.setRotation(rotation);

        return transform;
    }


    geometry_msgs::Point GetPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "body";
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

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
        geometry_msgs::TransformStamped transformStamped;
        std::string name = linkName.substr(0, linkName.size()-4);
        try{
            //transformStamped = tfBuffer.lookupTransform(name, "body", ros::Time(0));
            transformStamped = tfBuffer.lookupTransform("body", name, ros::Time(0));
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
        linkToCOMTransform.transform.rotation.x = q.getX();
        linkToCOMTransform.transform.rotation.y = q.getY();
        linkToCOMTransform.transform.rotation.z = q.getZ();
        linkToCOMTransform.transform.rotation.w = q.getW();
        if (linkName.compare("body_com") == 0)
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.01781, 0.0, 0.00882));
            linkToCOMTransform.transform.translation.x = 0.01781;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.00882;
        }
        else if (linkName.compare("tailbase_com") == 0)
        {
            // linkToCOMTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.04244));
            linkToCOMTransform.transform.translation.x = 0.0;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.04244;
        }
        else if (linkName.compare("tail_com") == 0)
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
            linkToCOMTransform.transform.translation.x = 0.10609;
            linkToCOMTransform.transform.translation.y = 0.0;
            linkToCOMTransform.transform.translation.z = 0.0;
        }

        tf2::Transform bodyToLinkTF2Transform, bodyToCOMTF2Transform, linkToCOMTF2Transform;
        tf2::convert(transformStamped, bodyToLinkTF2Transform);
        tf2::convert(linkToCOMTransform, linkToCOMTF2Transform);
        // tf2::fromMsg(transformStamped, bodyToLinkTF2Transform);
        // tf2::fromMsg(linkToCOMTransform, linkToCOMTF2Transform);
        bodyToCOMTF2Transform.mult(bodyToLinkTF2Transform, linkToCOMTF2Transform);
        //bodyToCOMTF2Transform.mult(linkToCOMTF2Transform, bodyToLinkTF2Transform);

        if (linkName.find("tibia") != std::string::npos)
        {
            auto origin = bodyToCOMTF2Transform.getOrigin();
            ROS_INFO("%s: (%f, %f, %f)", linkName.c_str(), origin.x(), origin.y(), origin.z());
        }

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
        double total_mass;

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
        total_mass = body_mass + 4*coxa_fr_mass + 4*femur_fr_mass + 4*tibia_fr_mass + tailbase_mass + tail_mass;

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
        COM.x = COM_x / total_mass;
        COM.y = COM_y / total_mass;
        COM.z = COM_z / total_mass;  
        return COM;
    }

    static bool sortByXCoord(geometry_msgs::Point point1, geometry_msgs::Point point2)
    {
        return (point1.x < point2.x);
    }

    void PublishStability()
    {
        geometry_msgs::Point foot_fr_pos, foot_fl_pos, foot_br_pos, foot_bl_pos;
        foot_fr_pos = GetPosition("foot_fr");
        foot_fl_pos = GetPosition("foot_fl");
        foot_br_pos = GetPosition("foot_br");
        foot_bl_pos = GetPosition("foot_bl");

        // find which legs are on the ground and create list of indices representing support structure (triangle or parallelogram)
        quadruped_control::VectorOfPoints supportStructure;
        if (frForce > 0)
            supportStructure.points.push_back(foot_fr_pos);
        if (flForce > 0)
            supportStructure.points.push_back(foot_fl_pos);
        if (brForce > 0)
            supportStructure.points.push_back(foot_br_pos);
        if (blForce > 0)
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

        std_msgs::Float64 marginMsg;
        marginMsg.data = stabilityMargin;
        stabilityMarginPublisher.publish(marginMsg);
    }

    double distancePointToEdge(geometry_msgs::Point COM_pos, geometry_msgs::Point point1, geometry_msgs::Point point2)
    {
        // solve for x distance only between COM and horizontal x intersection with line between point 1 and point 2
        return (COM_pos.y - point1.y)*(point2.x - point1.x)/(point2.y - point1.y) + point1.x - COM_pos.x;
    }

    ros::NodeHandle node;
    geometry_msgs::Point COM_pos;
    ros::Publisher COMPublisher;
    ros::Publisher supportStructurePublisher;
    ros::Publisher stabilityMarginPublisher;
    ros::ServiceClient linkStateClient;
    ros::ServiceClient linkPropertiesClient;
    ros::Subscriber frSensorSubscriber;
    ros::Subscriber flSensorSubscriber;
    ros::Subscriber brSensorSubscriber;
    ros::Subscriber blSensorSubscriber;
    ros::Timer timer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    double frForce;
    double flForce;
    double brForce;
    double blForce;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting COM Publisher Node...");
    ros::init(argc, argv, "com_publisher_node");
    ROS_INFO("Initialized ros...");

    ros::NodeHandle node;
    COMPublisherNode comPublisherNode(node);
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
