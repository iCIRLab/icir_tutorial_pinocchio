#include "icir_tutorial_pinocchio/icir_tutorial_pinocchio_sim.hpp"

using namespace std;
using namespace Eigen;
using namespace pinocchio;

int main(int argc, char **argv)
{
    /////////////////////////// Setting ////////////////////////////////////////////////    
    ros::init(argc, argv, "icir_gen3_pinocchio_sim");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(SAMPLING_RATE);

    // Mujoco Subs
    jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));

    // Mujoco Pubs
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_run_pub_ = n_node.advertise<std_msgs::Bool>("mujoco_ros/mujoco_ros_interface/sim_run", 5);

    //topic echo
    pos_des_pub_ = n_node.advertise<std_msgs::Float64MultiArray>("pos_des_topic", 10);
    pos_cur_pub_ = n_node.advertise<std_msgs::Float64MultiArray>("pos_cur_topic", 10);

    // Mujoco Msg
    robot_command_msg_.position.resize(GEN3_DOF); 
    robot_command_msg_.torque.resize(GEN3_DOF); 

    // Ros Param
    string urdf_name, urdf_path;
    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);    

    // Pinocchio
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;
    robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);  
    model_ = robot_->model();
    Data data(model_);
    data_ = data;

    // Control Variable
    ctrl_mode_ = 0;
    chg_flag_ = false;
    state_.J.resize(6, GEN3_DOF);   

    state_.q_des.setZero();
    state_.q_des_pre.setZero();   
    state_.v_des.setZero();
    state_.ddq_des.setZero();
    state_.tau_des.setZero();    

    m_p_.resize(12);
    m_v.resize(6);
    m_a.resize(6);
    m_v_ref = Motion(m_v.setZero());
    m_a_ref = Motion(m_a.setZero());
    m_frame_id = model_.getFrameId("Actuator6");    
    m_joint_id = model_.getJointId("Actuator6");    
    ////////////////////////////////////////////////////////////////////////////////////////      

    Home << 0.00, 45.0, 90.0, 0.0, 45.0, -90.0;
    Home2 << 45.00, 45.0, 90.0, 30.0, 50.0, 0.0;    
    ////////////////////////////////////////////////////////////////////////////////////////
            
    posture_Kp << 40000., 40000., 40000., 40000., 40000., 40000., 40000.;
    posture_Kd << 2.0*posture_Kp.cwiseSqrt();
    
    // ee_Kp << 1000., 1000., 1000., 2000., 2000., 2000.;
    ee_Kp << 5000., 5000., 5000., 50000., 50000., 50000.;
    ee_Kd << 2.0*ee_Kp.cwiseSqrt();
    ////////////////////////////////////////////////////////////////////////////////////////

    while (ros::ok()){
        keyboard_event();    
        
        std_msgs::String sim_run_msg_;
        sim_run_msg_.data = true;
        mujoco_run_pub_.publish(sim_run_msg_);
        
        robot_->computeAllTerms(data_, state_.q, state_.v);        

        if (ctrl_mode_== 0){
            state_.q_des.setZero();
            state_.tau_des.setZero();    
        }

        if (ctrl_mode_ == 1){ // joint task //h home
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                

                for (int i = 0; i<GEN3_DOF; i++)
                {
                    q_target_(i) = Home(i) * M_PI / 180.;
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                // state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), 0.0, 0.0);                            

                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -posture_Kp(i)*(state_.q(i) - state_.q_des(i)) -posture_Kd(i)*(state_.v(i) - state_.v_des(i));
            }                 
        }

        if (ctrl_mode_ == 2){ // joint task //a home2
            if (chg_flag_){
                cubic_.stime = time_;       
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                

                for (int i = 0; i<GEN3_DOF; i++)
                {                    
                    q_target_(i) = Home2(i) * M_PI / 180.;
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                // state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), 0.0, 0.0);                            

                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -posture_Kp(i)*(state_.q(i) - state_.q_des(i)) -posture_Kd(i)*(state_.v(i) - state_.v_des(i));
            }            
        }
        
        if (ctrl_mode_ == 3){ // ee task //k ee jog -0.05z
            if (chg_flag_){
                SE3Cubic_.stime = time_;
                SE3Cubic_.duration = 2.0;                

                //set init ee pose
                H_ee_ = robot_->position(data_, m_joint_id);
                SE3Cubic_.m_init = H_ee_;                  

                //set desired ee pose
                H_ee_ref_ = H_ee_;
                H_ee_ref_.translation()(2) -= 0.05;                        

                chg_flag_ = false;                
            }

            //desired trajectory [VectorXd];
            sampleEE_ = SE3Cubic(time_, SE3Cubic_.stime, SE3Cubic_.duration, SE3Cubic_.m_init, H_ee_ref_);
            vectorToSE3(sampleEE_, m_M_ref);  //desired trajectory in SE3 [pinocchio::SE3]                                 

            SE3 oMi;
            Motion v_frame;                                                            
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame
            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot            
            robot_->jacobianWorld(data_, m_joint_id, state_.J);           //jacobian in global frame     
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame
            
            SE3ToVector(oMi, m_p_);                                       // current pos in vector form            
            errorInSE3(oMi, m_M_ref, m_p_error);                          // pos erorr represented in local frame, oMi_inv*m_M_ref                                    

            // Transformation from local to world
            m_wMl.rotation(oMi.rotation());                               // use rotation only for vel&acc transformation

            m_p_error_vec = m_p_error.toVector();                         // pos err vector in local frame            
            m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;                 // vel err vector in local frame                         

            // desired acc in local frame
            m_a_des =   ee_Kp.cwiseProduct(m_p_error_vec)
                        + ee_Kd.cwiseProduct(m_v_error.toVector())
                        + m_wMl.actInv(m_a_ref).toVector();                             

            //transformation from ee to joint            
            state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                            

            //publish
            pos_des_pub();            
            pos_cur_pub();
        }

        if (ctrl_mode_ == 10){ // impedance control //v
            if (chg_flag_){
                SE3Cubic_.stime = time_;
                SE3Cubic_.duration = 2.0;                

                //set init ee pose
                H_ee_ = robot_->position(data_, m_joint_id);
                SE3Cubic_.m_init = H_ee_;                  

                //set desired ee pose
                H_ee_ref_ = H_ee_;                

                chg_flag_ = false;                
            }

            //to make K(x-xd)=0, put xd=x 
            H_ee_ref_.translation() = robot_->position(data_, m_joint_id).translation();
            m_M_ref = H_ee_ref_;
            
            // SE3ToVector(H_ee_ref_, sampleEE_);
            // vectorToSE3(sampleEE_, m_M_ref);  //desired trajectory in SE3 [pinocchio::SE3]                                 

            SE3 oMi;
            Motion v_frame;                                                            
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame
            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot                        
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame
            
            SE3ToVector(oMi, m_p_);                                       // current pos in vector form            
            errorInSE3(oMi, m_M_ref, m_p_error);                          // pos erorr represented in local frame, oMi_inv*m_M_ref                                    

            // Transformation from local to world
            m_wMl.rotation(oMi.rotation());                               // use rotation only for vel&acc transformation

            m_p_error_vec = m_p_error.toVector();                         // pos err vector in local frame            
            m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;                 // vel err vector in local frame                         

            cout << m_p_error_vec.transpose() << endl;

            // desired acc in local frame
            m_a_des =   ee_Kp.cwiseProduct(m_p_error_vec)
                        + ee_Kd.cwiseProduct(m_v_error.toVector())
                        + m_wMl.actInv(m_a_ref).toVector();                             

            //transformation from ee to joint            
            state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                            

            //publish
            pos_des_pub();            
            pos_cur_pub();
        }
    
        ////////////////////////////////////////////////////////////////////////////////////////
        state_.tau_des = data_.M * state_.ddq_des + data_.nle;
        robot_command(); //send torque command state_.tau_des to mujoco

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'h': //home joint
                ctrl_mode_ = 1;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home Position" << endl;
                cout << " " << endl;
                break;
            case 'g': //gravity
                ctrl_mode_ = 0;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "garvity mode" << endl;
                cout << " " << endl;
                break;
            case 'a': //a joint
                ctrl_mode_= 2;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home2 Position" << endl;
                cout << " " << endl;
                break;
            case 'k': //k ee task
                ctrl_mode_= 3;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee -0.05 z" << endl;
                cout << " " << endl;
                break;            
            case 'v': //v impedance control
                ctrl_mode_= 10;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "impedance control" << endl;
                cout << " " << endl;
                break;  
        }
    }
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  for (int i=0; i<GEN3_DOF; i++){
    state_.q(i) = msg->position[i];
    state_.v(i) = msg->velocity[i];
  }
}

void robot_command()
{
    robot_command_msg_.MODE = 1;
        robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    for (int i=0; i<GEN3_DOF; i++)
    {
        //robot_command_msg_.position[i] = state_.q_des(i);
        robot_command_msg_.torque[i] = state_.tau_des(i);
    }
    robot_command_pub_.publish(robot_command_msg_); 
}

void pos_des_pub()
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(sampleEE_.size());
    for(int i=0; i<sampleEE_.size(); i++){
        msg.data[i] = sampleEE_[i];
    }
    pos_des_pub_.publish(msg);
}

void pos_cur_pub()
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(m_p_.size());
    for(int i=0; i<m_p_.size(); i++){
        msg.data[i] = m_p_[i];
    }
    pos_cur_pub_.publish(msg);
}

