#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>
#include "eigen_conversions/eigen_kdl.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"

// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0);
std::vector<double> init_jnt_pos(7,0.0), aruco_pose(7,0.0);
bool robot_state_available = false, aruco_pose_available = false;
double lambda = 10*0.2;


#pragma region Functions
// Functions
KDLRobot createRobot(std::string robot_string)
{
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}
#pragma endregion

// Main
int main(int argc, char **argv)
{
    #pragma region Setup
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_7_effort_controller/command", 1);    

    ros::Publisher cart_error = n.advertise<std_msgs::Float64>("/iiwa/cart_error_topic", 1); 

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Set robot state
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(1.57);
    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg, cart_err_norm_msg;
    std_srvs::Empty pauseSrv;

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        if (!(robot_set_state_srv.call(robot_init_config)))
            ROS_INFO("Failed to set robot state.");            
        
        ros::spinOnce();
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    robot.update(jnt_pos, jnt_vel);
    int nrJnts = robot.getNrJnts();

    /*// Specify an end-effector 
    robot.addEE(KDL::Frame::Identity());*/

    // Specify an end-effector: camera in flange transform
    KDL::Frame ee_T_cam;
    ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57);
    ee_T_cam.p = KDL::Vector(0,0,0.025);
    robot.addEE(ee_T_cam);

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());
    qd.data.setZero();
    dqd.data.setZero();
    ddqd.data.setZero();

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Init controller
    KDLController controller_(robot);

    // EE's trajectory initial position
    KDL::Frame init_cart_pose = robot.getEEFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);

    /*// Retrieve initial ee pose
    KDL::Frame Fi = robot.getEEFrame();
    Eigen::Vector3d pdi = toEigen(Fi.p);*/

    // EE trajectory end position
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();
#pragma endregion
    
    int trajIndex = 2;  /*0)LinearTrap - enabled
                            ProfLinear - commented                         
                          1)CubicLinear - enabled
                            CubicLinear - commented
                          2)Circular Cubic
                          3)Circular Trap*/

    // Plan trajectory
    double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, init_time_slot = 1.0;
    double traj_radius = 0.1;

    //Constructors
    KDLPlanner planner(traj_duration, init_position, traj_radius, acc_duration); //Circular Trap & Cubic
    //KDLPlanner planner(traj_duration, acc_duration, init_position, end_position); // Lin Trap & Cubic

    // Gains
    double Kp;
    double Kd;

    // Retrieve the first trajectory point
    trajectory_point p;
    if(trajIndex == 0)
    {
        Kp = 20; 
        Kd = sqrt(Kp);
        p = planner.compute_LinTrapTrajectory(t); //Lin Trap
        //p = planner.compute_trajectory(t);
    }
    else if(trajIndex == 1)
    {
        Kp = 20;
        Kd = sqrt(Kp)-1;
        p = planner.compute_CubicLinearTrajectory(t); //CubicLinear
        //p = planner.compute_CubicLinearTrajectory2(t); //CubicLinear
    }
    else if(trajIndex == 2)
    {
        Kp = 20;
        Kd = sqrt(Kp);
        p = planner.compute_CircCubicTrajectory(t, traj_radius); //Circular Cubic
    }
    else if(trajIndex == 3)
    {
        Kp = 20;
        Kd = sqrt(Kp)+0.3;
        p = planner.compute_CircTrapTrajectory(t, traj_radius); //Circular Trap
    }

    
    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity(); KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
    des_pose.M = robot.getEEFrame().M;

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        if (robot_state_available)
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);

            // Update time
            t = (ros::Time::now()-begin).toSec();
            std::cout << "time: " << t << std::endl;            

            // Extract desired pose
            des_cart_vel = KDL::Twist::Zero();
            des_cart_acc = KDL::Twist::Zero();
            if (t <= init_time_slot) // wait a second
            {
                if(trajIndex == 0)
                {
                    p = planner.compute_LinTrapTrajectory(0.0);  //Lin Traj                    
                    //p = planner.compute_trajectory(0.0); //Prof Traj
                }
                else if(trajIndex == 1)
                {
                    p = planner.compute_CubicLinearTrajectory(0.0); //Cubic Lin Traj1
                    //p = planner.compute_CubicLinearTrajectory2(0.0); //Cubic Lin Traj2
                }
                else if(trajIndex == 2)
                {
                    p = planner.compute_CircCubicTrajectory(0.0, traj_radius); //Circular Traj
                }
                else if(trajIndex == 3)
                {
                    p = planner.compute_CircTrapTrajectory(0.0, traj_radius);
                }
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {
                if(trajIndex == 0)
                {
                    p = planner.compute_LinTrapTrajectory(t-init_time_slot);  //Lin Traj                    
                    //p = planner.compute_trajectory(t-init_time_slot); //Prof Traj
                }
                else if(trajIndex == 1)
                {
                    p = planner.compute_CubicLinearTrajectory(t-init_time_slot); //Cubic Lin Traj
                    //p = planner.compute_CubicLinearTrajectory2(t-init_time_slot); //Cubic Lin Traj2
                }
                else if(trajIndex == 2)
                {
                    p = planner.compute_CircCubicTrajectory(t-init_time_slot, traj_radius); //Circular Traj
                }
                else if(trajIndex == 3)
                {
                    p = planner.compute_CircTrapTrajectory(t-init_time_slot, traj_radius);
                }

                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }

            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);
            
            if(aruco_pose_available)
            {
                // compute current jacobians
                KDL::Jacobian J_cam = robot.getEEJacobian();
                KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));
                // look at point: compute rotation error from angle/axis
                Eigen::Matrix<double,3,1> aruco_pos_n = toEigen(cam_T_object.p); //(aruco_pose[0],aruco_pose[1],aruco_pose[2]);
                aruco_pos_n.normalize();
                Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;
                double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));
                KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);
                
                des_pose.M = robot.getEEFrame().M*Re; //ottengo la rotazione del des_pose moltiplicando il frame dell'end Effector per la matrice di rotazione
            }
            
            
            // std::cout << "jacobian: " << std::endl << robot.getEEJacobian().data << std::endl;
            // std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            // std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            // std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            // std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            // std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            // std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            // std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            // std::cout << "current_pose: " << std::endl << robot.getEEFrame() << std::endl;

            // inverse kinematics
            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
            qd = robot.getInvKin(qd, des_pose*robot.getFlangeEE().Inverse());
            robot.getInverseKinematics(des_pose, des_cart_vel, des_cart_acc,qd,dqd,ddqd);

            // joint space inverse dynamics control
            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);

            /*
            // Gains Operational Space Controller
            double Kp = 0;
            double Ko = 0;

            if(trajIndex == 0)
            {
                Kp=70;
                Ko=66;
                            
            // Cartesian space inverse dynamics control
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko, 1.7*sqrt(Kp), 2*sqrt(Ko));
            
            }
            else if(trajIndex == 1)
            {
                Kp=60;
                Ko=65;

            // Cartesian space inverse dynamics control
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko, 1.7*sqrt(Kp), 2*sqrt(Ko));
            }
            else if(trajIndex == 2)
            {
                Kp=40;
                Ko=38;
            
            // Cartesian space inverse dynamics control
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko, 1.3*sqrt(Kp), 1.5*sqrt(Ko));
            }
            else if(trajIndex == 3)
            {
                Kp=30;
                Ko=30;
            
            // Cartesian space inverse dynamics control
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Ko, 1.5*sqrt(Kp), 1.7*sqrt(Ko));

            }  
            */

            Eigen::MatrixXd errors = computeLinearError(toEigen(des_pose.p),toEigen(robot.getEEFrame().p));   
            //Eigen::VectorXd errors = qd.data-robot.getJntValues();                              

            // Set torques
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);

            cart_err_norm_msg.data = errors.norm();
            cart_error.publish(cart_err_norm_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");

    return 0;
}