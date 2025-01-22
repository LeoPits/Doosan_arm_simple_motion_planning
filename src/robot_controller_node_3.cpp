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

class RobotController {
public:
   RobotController(ros::NodeHandle& nh)
        : joint_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 10)),
          target_sub(nh.subscribe("target_position", 10, &RobotController::targetCallback, this)),
          joint_state_sub(nh.subscribe("joint_states", 10, &RobotController::jointStateCallback, this)),
          pose_pub(nh.advertise<geometry_msgs::Pose>("current_pose", 10))  // Nuovo publisher
    {
        joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Percorso al file URDF
        std::string urdf_path = ros::package::getPath("my_robot_controller");
        if (urdf_path.empty()) {
            ROS_ERROR("Il pacchetto my_robot_controller non è stato trovato.");
            return;
        }
        urdf_path += "/urdf/m0609.urdf";

        // Costruzione del modello KDL
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromFile(urdf_path, robot_tree)) {
            ROS_ERROR("Impossibile costruire l'albero KDL.");
            return;
        }

        // Estrai la catena del robot dalla struttura KDL
        if (!robot_tree.getChain("base", "link6", chain)) {
            ROS_ERROR("Impossibile ottenere la catena KDL.");
            return;
        }

        fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
        ik_solver = new KDL::ChainIkSolverPos_LMA(chain, 1e-5, 500);

        // Inizializza il messaggio dello stato dei giunti
        joint_state_msg.name = joint_names;
        joint_state_msg.position = joint_positions;
        publishJointStates();
        timer = nh.createTimer(ros::Duration(0.1), &RobotController::publishCurrentPose, this);

        
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        joint_positions = msg->position;
    }

    void targetCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        // Estrarre la posizione e l'orientamento dal messaggio ricevuto
        double x_target = msg->position.x;
        double y_target = msg->position.y;
        double z_target = msg->position.z;

        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll_target, pitch_target, yaw_target;
        tf2::Matrix3x3(q).getRPY(roll_target, pitch_target, yaw_target);

        // Crea una traiettoria verso la posizione target
        createTrajectoryToTarget(x_target, y_target, z_target, roll_target, pitch_target, yaw_target, 50);
    }

    bool inverseKinematics(double x, double y, double z, double roll, double pitch, double yaw, std::vector<double>& joint_angles) {
        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(x, y, z);
        target_frame.M = KDL::Rotation::RPY(roll, pitch, yaw);

        KDL::JntArray q_init(chain.getNrOfJoints());
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
            joint_angles.push_back(q_out(i));
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
            if (inverseKinematics(x, y, z, roll, pitch, yaw, joint_angles)) {
                trajectory.push_back(joint_angles);
            }
        }

        moveAlongTrajectory(trajectory, 10);
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
        // Calcola la cinematica diretta
        double x, y, z, roll, pitch, yaw;
        directKinematics(joint_positions, x, y, z, roll, pitch, yaw);

        // Crea un messaggio geometry_msgs::Pose
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

        // Pubblica la posizione corrente
        pose_pub.publish(current_pose);
    }


private:
    ros::Publisher joint_pub;
    ros::Subscriber joint_state_sub;
    ros::Subscriber target_sub;
    ros::Publisher pose_pub;  // Nuovo publisher per la posizione cartesiana
    ros::Timer timer;         // Timer per la pubblicazione periodica

    sensor_msgs::JointState joint_state_msg;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive* fk_solver;
    KDL::ChainIkSolverPos_LMA* ik_solver;
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
    controller.createTrajectoryToTarget(x_target, y_target, z_target, roll_target, pitch_target, yaw_target, 200);

    ros::spin();

    return 0;
}
