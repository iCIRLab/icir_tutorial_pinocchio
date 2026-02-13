//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

//Utility Header
#include "../utility/urdf_to_pin.hpp"
#include "../utility/math_functions.hpp"

//Mujoco MSG Header
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;
using namespace pinocchio;

#define SAMPLING_RATE 1000
#define GEN3_DOF 6

/////////////////////////// ROS Setting ////////////////////////////////////////////////    
ros::Publisher mujoco_command_pub_;
ros::Publisher robot_command_pub_;
ros::Publisher mujoco_run_pub_;
ros::Publisher pos_des_pub_, pos_cur_pub_;


ros::Subscriber jointState;
ros::Subscriber mujoco_command_sub;
ros::Subscriber mujoco_time_sub;

mujoco_ros_msgs::JointSet robot_command_msg_;

/////////////////////////// pinocchio & mujoco Setting /////////////////////////////////
typedef pinocchio::Model Model;
typedef pinocchio::Data Data;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

#if GEN3_DOF == 6    
    typedef struct State {   
        Vector6d q;
        Vector6d v;
        Vector6d q_des;
        Vector6d q_des_pre;
        Vector6d v_des;
        Vector6d ddq_des;
        Vector6d tau_des;
        Data::Matrix6x J;
    } state;  
    typedef struct CubicVar {
        Vector6d q0;
        Vector6d v0;
        double stime;
        double ftime;
    } cubicvar;
    typedef struct SE3CubicVar {
        pinocchio::SE3 m_init;
        pinocchio::SE3 m_goal;    
        double stime;
        double duration;
    } se3cubicvar;    
    Vector6d q_target_;    
    Vector6d posture_Kp, posture_Kd;
    Vector6d ee_Kp, ee_Kd;
#else
    typedef struct State {   
        Vector7d q;
        Vector7d v;
        Vector7d q_des;
        Vector7d q_des_pre;
        Vector7d v_des;
        Vector7d ddq_des;
        Vector7d tau_des;
        Data::Matrix6x J;
    } state;  
    typedef struct CubicVar {
        Vector7d q0;
        Vector7d v0;
        double stime;
        double ftime;
    } cubicvar;
    typedef struct SE3CubicVar {
        pinocchio::SE3 m_init;
        pinocchio::SE3 m_goal;    
        double stime;
        double duration;
    } se3cubicvar;    
    Vector7d q_target_;
    Vector7d posture_Kp, posture_Kd;
    Vector6d ee_Kp, ee_Kd;
#endif

// Pinocchio
std::shared_ptr<RobotWrapper> robot_;
Model model_;
Data data_;
SE3 H_ee_, H_ee_ref_;
Eigen::VectorXd m_p_;
pinocchio::Data::Matrix6x m_J_local_;

SE3 m_wMl;
Motion m_drift;            
Motion m_p_error, m_v_error, m_v_ref, m_a_ref;
Eigen::VectorXd m_v, m_a, m_p_error_vec;
Eigen::VectorXd m_v_ref_vec, m_a_des;
Model::JointIndex m_joint_id;
Model::FrameIndex m_frame_id;

// Control Variable
double mujoco_time_, time_;
state state_;
cubicvar cubic_;
se3cubicvar SE3Cubic_;
int ctrl_mode_;
bool chg_flag_;
Eigen::VectorXd sampleEE_;
pinocchio::SE3 m_M_ref;


// Waypoint
Vector6d Home, Home2;

void keyboard_event();
void pos_des_pub();
void pos_cur_pub();

bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
};

double wrapRadiansFromMinusPiToPi(double rad_not_wrapped)
{
    bool properly_wrapped = false;    
    do 
    {
        if (rad_not_wrapped > M_PI)
        {            
            rad_not_wrapped -= 2.0*M_PI;
        }
        else if (rad_not_wrapped < -M_PI)
        {         
            rad_not_wrapped += 2.0*M_PI;
        }
        else
        {
            properly_wrapped = true;
        }
    } while(!properly_wrapped);
    return rad_not_wrapped;
};

void simCommandCallback(const std_msgs::StringConstPtr &msg);
void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void robot_command();
