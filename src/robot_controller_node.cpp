#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>






class RobotController {
public:
    RobotController(ros::NodeHandle& nh)
        : joint_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 10)),
          target_sub(nh.subscribe("target_position", 10, &RobotController::targetCallback, this)),
          joint_state_sub(nh.subscribe("joint_states", 10, &RobotController::jointStateCallback, this)),
          pose_pub(nh.advertise<geometry_msgs::Pose>("current_pose", 10)),
          trajectory_pub(nh.advertise<visualization_msgs::Marker>("trajectory_marker", 10)) // New publisher for visualization
    {
        joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        urdf::Model robot_model;
        std::string urdf_path = ros::package::getPath("my_robot_controller") + "/urdf/m0609.urdf";

        if (!robot_model.initFile(urdf_path)) {
            ROS_ERROR("Failed to load URDF model from file %s", urdf_path.c_str());
            return;
        }

        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromFile(urdf_path, robot_tree)) {
            ROS_ERROR("Failed to construct KDL tree.");
            return;
        }

        if (!robot_tree.getChain("base", "link6", chain)) {
            ROS_ERROR("Failed to extract KDL chain.");
            return;
        }

        joint_limits_lower.resize(chain.getNrOfJoints());
        joint_limits_upper.resize(chain.getNrOfJoints());

        size_t joint_index = 0;
        for (const auto& segment : chain.segments) {
            const std::string joint_name = segment.getJoint().getName();
            auto joint = robot_model.getJoint(joint_name);
            if (joint && joint->limits) {
                joint_limits_lower[joint_index] = joint->limits->lower;
                joint_limits_upper[joint_index] = joint->limits->upper;
                joint_index++;
            }
        }
        fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
        ik_solver = new KDL::ChainIkSolverPos_LMA(chain, 1e-5, 1000);

        joint_state_msg.name = joint_names;
        joint_state_msg.position = joint_positions;
        publishJointStates();
        timer = nh.createTimer(ros::Duration(0.1), &RobotController::publishCurrentPose, this);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        joint_positions = msg->position;
    }

    void targetCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        double x_target = msg->position.x;
        double y_target = msg->position.y;
        double z_target = msg->position.z;

        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        q.normalize();

        double roll_target, pitch_target, yaw_target;
        tf2::Matrix3x3(q).getRPY(roll_target, pitch_target, yaw_target);

        if (!isWithinWorkspace(x_target, y_target, z_target)) {
            ROS_ERROR("Target is out of the workspace.");
            return;
        }
        createTrajectoryToTarget(x_target, y_target, z_target, roll_target, pitch_target, yaw_target, 200);
    }

    bool isWithinWorkspace(double x, double y, double z) {
        KDL::Vector target(x, y, z);
        double distance_to_base = target.Norm();

        double max_reach = 0.0;
        for (const auto& segment : chain.segments) {
            //max_reach += segment.getSegment().getFrameToTip().p.Norm();
            max_reach += segment.pose(0).p.Norm();

        }

        return distance_to_base <= max_reach;
    }

    bool safeInverseKinematics(double x, double y, double z, double roll, double pitch, double yaw, std::vector<double>& joint_angles) {
        double step = 0.01;
        double threshold = 0.2;
        double factor = 1.0;

        while (factor > threshold) {
            if (inverseKinematics(x * factor, y * factor, z * factor, roll, pitch, yaw, joint_angles)) {
                return true;
            }
            factor -= step;
        }

        ROS_ERROR("Safe IK failed.");
        return false;
    }

    bool inverseKinematics(double x, double y, double z, double roll, double pitch, double yaw, std::vector<double>& joint_angles) {
        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(x, y, z);
        target_frame.M = KDL::Rotation::RPY(roll, pitch, yaw);

        KDL::JntArray  q_init(chain.getNrOfJoints());
        KDL::JntArray q_out(chain.getNrOfJoints());

        for (int i = 0; i < chain.getNrOfJoints(); i++) {
            q_init(i) = joint_positions[i];
        }

        int result = ik_solver->CartToJnt(q_init, target_frame, q_out);
        if (result < 0) {
            ROS_ERROR("IK solver failed.");
            return false;
        }

        joint_angles.clear();
        for (int i = 0; i < chain.getNrOfJoints(); i++) {
            double angle = q_out(i);
            angle = std::max(joint_limits_lower[i], std::min(joint_limits_upper[i], angle));
            joint_angles.push_back(angle);
        }
        return true;
    }

    void createTrajectoryToTarget(double x_target, double y_target, double z_target, double roll_target, double pitch_target, double yaw_target, size_t num_steps) {
        double x_start, y_start, z_start, roll_start, pitch_start, yaw_start;
        directKinematics(joint_positions, x_start, y_start, z_start, roll_start, pitch_start, yaw_start);

        std::vector<std::vector<double>> trajectory;
        for (size_t i = 0; i < num_steps; ++i) {
            double t = static_cast<double>(i) / (num_steps - 1);
            double x = x_start + t * (x_target - x_start);
            double y = y_start + t * (y_target - y_start);
            double z = z_start + t * (z_target - z_start);
            double roll = roll_start + t * (roll_target - roll_start);
            double pitch = pitch_start + t * (pitch_target - pitch_start);
            double yaw = yaw_start + t * (yaw_target - yaw_start);

            std::vector<double> joint_angles;
            if (!safeInverseKinematics(x, y, z, roll, pitch, yaw, joint_angles)) {
                ROS_WARN("Skipping step %zu: Safe IK failed for target pose.", i);
                continue;
            }
            trajectory.push_back(joint_angles);
        }

        visualizeTrajectory(trajectory);
        moveAlongTrajectory(trajectory, 10);
    }

    void visualizeTrajectory(const std::vector<std::vector<double>>& trajectory) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_0";
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        for (const auto& joint_angles : trajectory) {
            double x, y, z, roll, pitch, yaw;
            directKinematics(joint_angles, x, y, z, roll, pitch, yaw);
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            marker.points.push_back(p);
        }
        trajectory_pub.publish(marker);

    }
    

    void moveAlongTrajectory(const std::vector<std::vector<double>>& trajectory, double duration) {
        ros::Rate rate(50);
        double time_step = duration / trajectory.size();

        for (const auto& joint_angles : trajectory) {
            joint_positions = joint_angles;
            publishJointStates();
            rate.sleep();
        }
    }

    void directKinematics(const std::vector<double>& joint_angles, double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
        KDL::JntArray joint_positions_kdl(chain.getNrOfJoints());
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            joint_positions_kdl(i) = joint_angles[i];
        }

        KDL::Frame end_effector_frame;
        fk_solver->JntToCart(joint_positions_kdl, end_effector_frame);

        x = end_effector_frame.p.x();
        y = end_effector_frame.p.y();
        z = end_effector_frame.p.z();
        end_effector_frame.M.GetRPY(roll, pitch, yaw);
    }

    void publishJointStates() {
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.position = joint_positions;
        joint_pub.publish(joint_state_msg);
    }

    void publishCurrentPose(const ros::TimerEvent&) {
        double x, y, z, roll, pitch, yaw;
        directKinematics(joint_positions, x, y, z, roll, pitch, yaw);

        geometry_msgs::Pose current_pose;
        current_pose.position.x = x;
        current_pose.position.y = y;
        current_pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        current_pose.orientation.x = q.x();
        current_pose.orientation.y = q.y();
        current_pose.orientation.z = q.z();
        current_pose.orientation.w = q.w();

        pose_pub.publish(current_pose);
    }

private:
    ros::Publisher joint_pub;
    ros::Subscriber target_sub;
    ros::Subscriber joint_state_sub;
    ros::Publisher pose_pub;
    ros::Publisher trajectory_pub; // Publisher for trajectory visualization
    ros::Timer timer;

    sensor_msgs::JointState joint_state_msg;

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive* fk_solver;
    KDL::ChainIkSolverPos_LMA* ik_solver;

    std::vector<double> joint_limits_lower;
    std::vector<double> joint_limits_upper;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh;

    RobotController controller(nh);

    ROS_INFO("Robot Controller Node Started.");

    // Esempio di target da raggiungere
    double x_target = 0.5, y_target = 0.2, z_target = 0.5;
    double roll_target = 1.7, pitch_target = 0.7, yaw_target = 1.0;

    // Crea la traiettoria per il movimento
    controller.createTrajectoryToTarget(x_target, y_target, z_target, roll_target, pitch_target, yaw_target, 150);

    ros::spin();

    return 0;
}
