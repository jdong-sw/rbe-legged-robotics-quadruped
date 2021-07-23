#include <ros/ros.h>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "quadruped_control/SolveIKPose.h"
#include "quadruped_control/SolveFKPose.h"
#include "quadruped_control/Pose.h"

namespace quadruped_control
{
    class KinematicsSolver
    {
    public:
        KinematicsSolver(ros::NodeHandle *node, KDL::Tree tree)
        {
            this->node = node;

            KDL::Chain chain;
            tree.getChain("body", "tailtip", this->tail);
            tree.getChain("body", "foot_fr", this->leg_fr);
            tree.getChain("body", "foot_fl", this->leg_fl);
            tree.getChain("body", "foot_br", this->leg_br);
            tree.getChain("body", "foot_bl", this->leg_bl);

            ROS_INFO("Initializing IKPoseSolver service...");
            this->ikTailPoseService = node->advertiseService("/quadruped/tail/ik", &KinematicsSolver::solveIKPoseTail, this);
            this->ikFRPoseService = node->advertiseService("/quadruped/leg_fr/ik", &KinematicsSolver::solveIKPoseLegFR, this);
            this->ikFLPoseService = node->advertiseService("/quadruped/leg_fl/ik", &KinematicsSolver::solveIKPoseLegFL, this);
            this->ikBRPoseService = node->advertiseService("/quadruped/leg_br/ik", &KinematicsSolver::solveIKPoseLegBR, this);
            this->ikBLPoseService = node->advertiseService("/quadruped/leg_bl/ik", &KinematicsSolver::solveIKPoseLegBL, this);

            ROS_INFO("Initializing FKPoseSolver service...");
            this->fkTailPoseService = node->advertiseService("/quadruped/tail/fk", &KinematicsSolver::solveFKPoseTail, this);
            this->fkFRPoseService = node->advertiseService("/quadruped/leg_fr/fk", &KinematicsSolver::solveFKPoseLegFR, this);
            this->fkFLPoseService = node->advertiseService("/quadruped/leg_fl/fk", &KinematicsSolver::solveFKPoseLegFL, this);
            this->fkBRPoseService = node->advertiseService("/quadruped/leg_br/fk", &KinematicsSolver::solveFKPoseLegBR, this);
            this->fkBLPoseService = node->advertiseService("/quadruped/leg_bl/fk", &KinematicsSolver::solveFKPoseLegBL, this);

            ROS_INFO("Ready.");
        }

        ~KinematicsSolver()
        {
            this->node->shutdown();
        }

        bool solveIKPoseTail(quadruped_control::SolveIKPoseRequest &req,
                         quadruped_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->tail);
        }

        bool solveIKPoseLegFR(quadruped_control::SolveIKPoseRequest &req,
                         quadruped_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_fr);
        }

        bool solveIKPoseLegFL(quadruped_control::SolveIKPoseRequest &req,
                         quadruped_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_fl);
        }

        bool solveIKPoseLegBR(quadruped_control::SolveIKPoseRequest &req,
                         quadruped_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_br);
        }

        bool solveIKPoseLegBL(quadruped_control::SolveIKPoseRequest &req,
                         quadruped_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_bl);
        }

        bool solveIKPose(quadruped_control::SolveIKPoseRequest &req,
                         quadruped_control::SolveIKPoseResponse &res,
                         KDL::Chain chain)
        {
            ROS_DEBUG("Solving IK Pose...");

            ROS_DEBUG("Initializing solver...");

            Eigen::Matrix<double, 6, 1> weights;
            weights.col(0) << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

            KDL::ChainIkSolverPos_LMA posSolver(chain, weights);
            std::vector<double> initialstate = req.initialState;

            KDL::JntArray jntArray = KDL::JntArray(3);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(initialstate.data(), initialstate.size());

            ROS_DEBUG("Parsing rotation...");
            KDL::Vector rotx = KDL::Vector(req.goal.rotx[0], req.goal.rotx[1], req.goal.rotx[2]);
            KDL::Vector roty = KDL::Vector(req.goal.roty[0], req.goal.roty[1], req.goal.roty[2]);
            KDL::Vector rotz = KDL::Vector(req.goal.rotz[0], req.goal.rotz[1], req.goal.rotz[2]);

            ROS_DEBUG("Parsing posiion...");
            KDL::Vector targetpos = KDL::Vector(req.goal.x, req.goal.y, req.goal.z);
            KDL::Rotation targetrot = KDL::Rotation(rotx, roty, rotz);

            ROS_DEBUG("Generating goal...");
            KDL::Frame targetFrame = KDL::Frame(targetrot, targetpos);

            KDL::JntArray jntSol = KDL::JntArray(3);
            ROS_DEBUG("[%f, %f, %f]; [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]",
                        targetpos.data[0], targetpos.data[1], targetpos.data[2],
                        rotx.data[0], roty.data[0], rotz.data[0],
                        rotx.data[1], roty.data[1], rotz.data[1],
                        rotx.data[2], roty.data[2], rotz.data[2]);

            ROS_DEBUG("Running solver...");
            int result = posSolver.CartToJnt(jntArray, targetFrame, jntSol);
            ROS_DEBUG("Result: %d", result);

            for (int i = 0; i < jntSol.rows() * jntSol.columns(); i++)
            {
                ROS_DEBUG("%f", jntSol.data[i]);
            }

            std::vector<double> sol(jntSol.data.data(), jntSol.data.data() + jntSol.data.rows() * jntSol.data.cols());

            res.solution = sol;
            res.result = result;
            res.error_message = std::string(posSolver.strError(res.result));
            return true;
        }

        bool solveFKPoseTail(quadruped_control::SolveFKPoseRequest &req,
                         quadruped_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->tail);
        }

        bool solveFKPoseLegFR(quadruped_control::SolveFKPoseRequest &req,
                         quadruped_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_fr);
        }

        bool solveFKPoseLegFL(quadruped_control::SolveFKPoseRequest &req,
                         quadruped_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_fl);
        }

        bool solveFKPoseLegBR(quadruped_control::SolveFKPoseRequest &req,
                         quadruped_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_br);
        }

        bool solveFKPoseLegBL(quadruped_control::SolveFKPoseRequest &req,
                         quadruped_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_bl);
        }

        bool solveFKPose(quadruped_control::SolveFKPoseRequest &req,
                         quadruped_control::SolveFKPoseResponse &res,
                         KDL::Chain chain)
        {
            ROS_DEBUG("FK Pose request received.");
            KDL::ChainFkSolverPos_recursive posSolver(chain);

            ROS_DEBUG("Joint positions:");
            std::vector<double> jointPositions = req.jointPositions;
            for (int i = 0; i < jointPositions.size(); ++i)
            {
                ROS_DEBUG("  %f", jointPositions[i]);
            }

            KDL::JntArray jntArray = KDL::JntArray(7);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(jointPositions.data(), jointPositions.size());

            KDL::Frame frame = KDL::Frame();

            int result = posSolver.JntToCart(jntArray, frame);
            res.result = result;
            res.error_message = std::string(posSolver.strError(res.result));

            res.solution.x = frame.p.data[0];
            res.solution.y = frame.p.data[1];
            res.solution.z = frame.p.data[2];

            std::vector<double> rotx = std::vector<double> {
                frame.M.UnitX().data[0],
                frame.M.UnitX().data[1],
                frame.M.UnitX().data[2]
            };
            
            std::vector<double> roty = std::vector<double> {
                frame.M.UnitY().data[0],
                frame.M.UnitY().data[1],
                frame.M.UnitY().data[2]
            };

            std::vector<double> rotz = std::vector<double> {
                frame.M.UnitZ().data[0],
                frame.M.UnitZ().data[1],
                frame.M.UnitZ().data[2]
            };

            res.solution.rotx = rotx;
            res.solution.roty = roty;
            res.solution.rotz = rotz;

            ROS_DEBUG("Solved position: {%f, %f, %f}", res.solution.x, res.solution.y, res.solution.z);
            ROS_DEBUG("Solved rotx: {%f, %f, %f}", rotx[0], rotx[1], rotx[2]);
            ROS_DEBUG("Solved roty: {%f, %f, %f}", roty[0], roty[1], roty[2]);
            ROS_DEBUG("Solved rotz: {%f, %f, %f}", rotz[0], rotz[1], rotz[2]);

            return true;
        }

    private:
        ros::NodeHandle *node;
        ros::ServiceServer ikFRPoseService;
        ros::ServiceServer ikFLPoseService;
        ros::ServiceServer ikBRPoseService;
        ros::ServiceServer ikBLPoseService;
        ros::ServiceServer ikTailPoseService;
        ros::ServiceServer fkFRPoseService;
        ros::ServiceServer fkFLPoseService;
        ros::ServiceServer fkBRPoseService;
        ros::ServiceServer fkBLPoseService;
        ros::ServiceServer fkTailPoseService;

        KDL::Chain tail;
        KDL::Chain leg_fr;
        KDL::Chain leg_fl;
        KDL::Chain leg_br;
        KDL::Chain leg_bl;
    };
} // namespace quadruped_control

int main(int argc, char **argv)
{
    ROS_INFO("Starting Inverse Kinematics solver...");
    ROS_INFO("Building robot tree from param server...");
    ros::init(argc, argv, "kinematics_solver");
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("/robot_description", robot_desc_string, std::string());
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_INFO("Failed to construct kdl tree");
        return 0;
    }
    else
    {
        ROS_INFO("Build chain of %d segments", tree.getNrOfSegments());
    }

    KDL::SegmentMap::const_iterator it;
    for (it = tree.getSegments().begin(); it != tree.getSegments().end(); it++)
    {
        ROS_INFO("Segment %s", it->second.segment.getName().c_str());
    }

    quadruped_control::KinematicsSolver solver(&node, tree);

    //ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}