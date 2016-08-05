#include <hros5_ros_control/hros5_hardware_interface.h>
#include <angles/angles.h>
#include <math.h>

#include <std_msgs/Float64.h>

namespace hros5
{
using namespace Robot;

const std::string RobotHardwareInterface::jointUIDs[JointData::NUMBER_OF_JOINTS+3] =
{
    "RShoulderPitch",
    "LShoulderPitch",
    "RShoulderRoll",
    "LShoulderRoll",
    "RElbowPitch",
    "LElbowPitch",
    "RHipYaw",
    "LHipYaw",
    "RHipRoll",
    "LHipRoll",
    "RHipPitch",
    "LHipPitch",
    "RKneePitch",
    "LKneePitch",
    "RAnklePitch",
    "LAnklePitch",
    "RAnkleRoll",
    "LAnkleRoll",
    "HeadYaw",
    "HeadPitch",
    "j_wrist_r",
    "j_wrist_l",
    "j_gripper_r",
    "j_gripper_l",
};


const int RobotHardwareInterface::ros_joint_offsets[JointData::NUMBER_OF_JOINTS+3] =
{
    0,        //RShoulderPitch
    0,        //LShoulderPitch
    0,        //RShoulderRoll
    0,        //LShoulderRoll
    0,        //RElbowPitch
    0,        //LElbowPitch
    0,        //RHipYaw
    0,        //LHipYaw
    0,        //RHipRoll
    0,        //LHipRoll
    0,        //RHipPitch
    0,        //LHipPitch
    0,        //RKneePitch
    0,        //LKneePitch
    0,        //RAnklePitch
    0,        //LAnklePitch
    0,        //RAnkleRoll
    0,        //LAnkleRoll
    0,        //HeadYaw
    0,        //HeadPitch
    0,        //j_wrist_r
    0,        //j_wrist_l
    0,        //j_gripper_r
    0,        //j_gripper_l
};

RobotHardwareInterface::Ptr RobotHardwareInterface::singelton = RobotHardwareInterface::Ptr();

RobotHardwareInterface::RobotHardwareInterface()
    : MotionModule()
    , hardware_interface::RobotHW()
    , joint_state_intervall_(30.0)
    , last_joint_state_read_(ros::Time::now())
{
    imu_gyro_calibrated_ = false;
    print_check_fall_debug_info_ = false;
    block_write_ = true;
    controller_running_ = false;

    ros::NodeHandle nh;
    int wakeup_motion;
    nh.getParam("/hros5/hros5_ros_controller/wake_up_motion",wakeup_motion);

    // Initialize Original Framework
    cm730_device_ = std::string("/dev/arbotix");
    nh.getParam("/hros5/hros5_ros_controller/hros5_config_file",config_file_);
    nh.getParam("/hros5/hros5_ros_controller/hros5_action_file",action_file_);

    if(false == Action::GetInstance()->LoadFile((char *)action_file_.c_str()))
    {
        ROS_FATAL("Reading Action File failed!");
        ros::shutdown();
        exit(-1);
    }

    linux_cm730_ = new LinuxArbotixPro((char *)cm730_device_.c_str());
    cm730_ = new ArbotixPro(linux_cm730_);
    MotionManager::GetInstance();

    if(false == MotionManager::GetInstance()->Initialize(cm730_))
    {
        ROS_FATAL("Initializing Motion Manager failed!");
        ros::shutdown();
        exit(-1);
    }

    minIni ini =  minIni(config_file_);
    MotionManager::GetInstance()->LoadINISettings(&ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());

    motion_timer_ = new LinuxMotionTimer();
    motion_timer_->Initialize(MotionManager::GetInstance());
    ROS_INFO("Starting Motion Timer...");
    motion_timer_->Start();
    ROS_INFO("Finished starting Motion Timer");

    MotionManager::GetInstance()->SetEnable(true);

    /** Init(stand up) pose */
    ROS_INFO("Wake up position is %i",wakeup_motion);
    if(Action::GetInstance()->Start(wakeup_motion))
        ROS_INFO("Moving to wake up position ...");
    else
        ROS_ERROR("Wake up action failed");
    while(Action::GetInstance()->IsRunning())
    {
        //usleep(8*1000);
        ros::Duration(0.008).sleep();
    }

    Walking::GetInstance()->LoadINISettings( &ini );
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionStatus::m_CurrentJoints.SetEnableBody(true,true);

    /** register joints */
    // dispatching joints
    for (unsigned int id_index = 0; id_index < JointData::NUMBER_OF_JOINTS-1; id_index++)
    {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id_index], &pos_[id_index], &vel_[id_index], &eff_dummy_[id_index]);
        joint_state_interface_.registerHandle(joint_state_handle);
        // connect and register the joint position interface
        hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd_[id_index]);
        pos_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&pos_joint_interface_);

    /** register sensors */
    // IMU
//TODO: Parameterize name and frame_id for arbotix/cm intertial sensor
    imu_data_.name = "imu_arbotix";
    imu_data_.frame_id = "base_link";
    imu_data_.orientation = imu_orientation_;
    imu_data_.angular_velocity = imu_angular_velocity_;
    imu_data_.linear_acceleration = imu_linear_acceleration_;
    hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    registerInterface(&imu_sensor_interface_);

    imu_timestamp = ros::Time::now();

    filtered_pitch = 0.0;
    filtered_roll = 0.0;
    imu_angular_velocity_[0] = 0.0;
    imu_angular_velocity_[1] = 0.0;
    imu_angular_velocity_[2] = 0.0;
    imu_linear_acceleration_[0] = 0.0;
    imu_linear_acceleration_[1] = 0.0;
    imu_linear_acceleration_[2] = 0.0;

    is_fallen_pub_ = nh.advertise<std_msgs::Bool>("/hros5/isFallen", 100);
    is_walking_pub_ = nh.advertise<std_msgs::Bool>("/hros5/isWalking", 100);

}

RobotHardwareInterface::RobotHardwareInterface(RobotHardwareInterface const&)
{
}

RobotHardwareInterface::~RobotHardwareInterface()
{
    delete(linux_cm730_);
    delete(cm730_);
    delete(motion_timer_);
}

RobotHardwareInterface& RobotHardwareInterface::operator=(RobotHardwareInterface const&)
{
}

RobotHardwareInterface::Ptr& RobotHardwareInterface::Instance()
{
    if (!singelton)
        singelton.reset(new RobotHardwareInterface());
    return singelton;
}

void RobotHardwareInterface::Initialize()
{
}

void RobotHardwareInterface::Process()
{
}

double lowPassFilter(double alpha, double x_new, double x_old)
{
    return alpha*x_new + (1.0-alpha)*x_old;
}

void RobotHardwareInterface::read(ros::Time time, ros::Duration period)
{
    for (unsigned int i = 0; i < JointData::NUMBER_OF_JOINTS-1; i++)
    {
        cmd_[i] = std::numeric_limits<double>::quiet_NaN();
        pos_[i] = std::numeric_limits<double>::quiet_NaN();
        vel_[i] = std::numeric_limits<double>::quiet_NaN();
        eff_dummy_[i] = std::numeric_limits<double>::quiet_NaN();
    }

    for (unsigned int joint_index = 1; joint_index < JointData::NUMBER_OF_JOINTS; joint_index++)
    {
        int id_index = joint_index-1;
        double new_pos = angles::from_degrees(MotionStatus::m_CurrentJoints.GetAngle(joint_index)+ros_joint_offsets[joint_index]);
        vel_[id_index] = (new_pos - pos_[id_index])/(double)period.toSec();
        pos_[id_index]= new_pos;
        //reading velocity and acceleration with the current firmware is not possible without jamming the cm730, state 05/2015
    }

    //IMU
    double filter_alpha = 0.08;
    double filter_alpha_gyro = 0.08;

//TODO: NOTE: We need a properly calibrated gyro value on each axis. Can we do this in Arbotix? Better yet, provide accurate pitch and roll from arbotix directly.
if ( imu_gyro_calibrated_ < 100 )
{
    imu_angular_velocity_[0] = lowPassFilter(filter_alpha,(-cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_GYRO_X_L)-512)*1600.0*M_PI/(512.0*180.0)*0.008,imu_angular_velocity_[0]);
    imu_angular_velocity_[1] = lowPassFilter(filter_alpha,(-cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_GYRO_Y_L)-512)*1600.0*M_PI/(512.0*180.0)*0.008,imu_angular_velocity_[1]);
    imu_angular_velocity_[2] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_GYRO_Z_L)-512)*1600.0*M_PI/(512.0*180.0)*0.008,imu_angular_velocity_[2]);    
    ++imu_gyro_calibrated_;
    if ( imu_gyro_calibrated_ == 100 )
    {
        imu_gyro_zero_[0] = imu_angular_velocity_[0];
        imu_gyro_zero_[1] = imu_angular_velocity_[1];
        imu_gyro_zero_[2] = imu_angular_velocity_[2];
        ROS_INFO( "Gyro Zero Offset Calibration: GyroX:\t%0.4f\tGyroY:\t%0.4f\tGyroZ:\t%0.4f", imu_gyro_zero_[0], imu_gyro_zero_[1], imu_gyro_zero_[2] );
        imu_timestamp = ros::Time::now();
    }

    return;
}

    //in rad/s
    //const double TICKS_TO_RADIANS_PER_STEP = (M_PI / 180.0) * 250.0 / 512.0 * (0.001 * MotionModule::TIME_UNIT)
    imu_angular_velocity_[0] = lowPassFilter(filter_alpha_gyro,((-cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_GYRO_X_L)-512)*1600.0*M_PI/(512.0*180.0)*0.008)-imu_gyro_zero_[0],imu_angular_velocity_[0]);
    imu_angular_velocity_[1] = lowPassFilter(filter_alpha_gyro,((-cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_GYRO_Y_L)-512)*1600.0*M_PI/(512.0*180.0)*0.008)-imu_gyro_zero_[1],imu_angular_velocity_[1]);
    imu_angular_velocity_[2] = lowPassFilter(filter_alpha_gyro,((cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_GYRO_Z_L)-512)*1600.0*M_PI/(512.0*180.0)*0.008)-imu_gyro_zero_[2],imu_angular_velocity_[2]);
//ROS_INFO( "GyroX:\t%0.4f\tGyroY:\t%0.4f\tGyroZ:\t%0.4f", imu_angular_velocity_[0], imu_angular_velocity_[1], imu_angular_velocity_[2] );

    //in m/s^2
    imu_linear_acceleration_[0] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_ACCEL_Y_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[0]);
    imu_linear_acceleration_[1] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_ACCEL_X_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[1]);
    imu_linear_acceleration_[2] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[ArbotixPro::ID_CM].ReadWord(ArbotixPro::P_ACCEL_Z_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[2]);
//ROS_INFO( "AccelX: %0.4f AccelY: %0.4f AccelZ: %0.4f", imu_linear_acceleration_[0], imu_linear_acceleration_[1], imu_linear_acceleration_[2] );

    //Estimation of roll and pitch based on accelometer data, see http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
    double sign = copysignf(1.0,  imu_linear_acceleration_[2]/G_ACC);
    double roll = -atan2( imu_linear_acceleration_[1]/G_ACC, sign * sqrt( imu_linear_acceleration_[0]/G_ACC* imu_linear_acceleration_[0]/G_ACC +  imu_linear_acceleration_[2]/G_ACC* imu_linear_acceleration_[2]/G_ACC));
    double pitch = atan2( imu_linear_acceleration_[0]/G_ACC, sqrt( imu_linear_acceleration_[1]/G_ACC* imu_linear_acceleration_[1]/G_ACC +  imu_linear_acceleration_[2]/G_ACC* imu_linear_acceleration_[2]/G_ACC));
    double yaw = 0.0;

//ROS_INFO( "pitch: %0.2fdeg roll: %0.2fdeg", pitch*57.2958, roll*57.2958 );

    //TODO: Testing complimentary filter
    ros::Duration duration = ros::Time::now() - imu_timestamp;
    imu_timestamp = ros::Time::now();
    filtered_pitch = 0.9 * ( filtered_pitch + ( duration.toSec() * imu_angular_velocity_[1] ) ) + ( 0.1 * pitch );
    filtered_roll = 0.9 * ( filtered_roll + ( duration.toSec() * imu_angular_velocity_[0] ) ) + ( 0.1 * roll );

//ROS_INFO( "filtered_pitch: %0.2fdeg filtered_roll: %0.2fdeg", filtered_pitch*57.2958, filtered_roll*57.2958 );

    tf2::Quaternion imu_orient;
    imu_orient.setEuler(filtered_pitch, /*filtered_roll*/ 0.0, 0.0); //TODO: Quaternion.setEuler(yaw, pitch, roll)?
    imu_orientation_[0] = imu_orient.getX();
    imu_orientation_[1] = imu_orient.getY();
    imu_orientation_[2] = imu_orient.getZ();
    imu_orientation_[3] = imu_orient.getW();
}

void RobotHardwareInterface::write(ros::Time time, ros::Duration period)
{
    if(!controller_running_)
    {
        if (!std::isnan(cmd_[JointData::NUMBER_OF_JOINTS-1]))
        {
            setBlockWrite(false);
            controller_running_=true;
            return;
        }
    }

    if(block_write_)
    {
        return;
    }

    for (unsigned int joint_index = 0; joint_index < JointData::NUMBER_OF_JOINTS-1; joint_index++)
    {
        int id_index = joint_index+1;
        if(MotionStatus::m_CurrentJoints.GetEnable(id_index))
        {
            if (std::isnan(cmd_[joint_index]))
            {
                //ROS_WARN("Cmd %i NAN", joint_index);
                continue;
            }
            else
            {
                MotionStatus::m_CurrentJoints.SetAngle(id_index,angles::to_degrees(cmd_[joint_index])+ros_joint_offsets[joint_index]);
            }
        }
    }
}


void RobotHardwareInterface::setPIDGains(int id, double p_gain, double i_gain, double d_gain)
{
    MotionStatus::m_CurrentJoints.SetPGain(id, p_gain);
    MotionStatus::m_CurrentJoints.SetIGain(id, i_gain);
    MotionStatus::m_CurrentJoints.SetDGain(id, d_gain);
}


void RobotHardwareInterface::setJointStateRate(double joint_state_rate)
{
    this->joint_state_intervall_ = 1.0/joint_state_rate;
}

void RobotHardwareInterface::setTorqueOn(int id, bool enable)
{
    int error;
    cm730_->WriteByte(id, MXDXL::P_TORQUE_ENABLE, enable ? 1 : 0, &error);
}

void RobotHardwareInterface::setTorqueOn(bool enable)
{
    if (enable)
        ROS_INFO("Enable torque!");
    else
        ROS_INFO("Disable torque!");

    int error;
    cm730_->WriteByte(ArbotixPro::ID_BROADCAST, MXDXL::P_TORQUE_ENABLE, enable, &error);
}

void RobotHardwareInterface::cmdWalking(const geometry_msgs::Twist::ConstPtr& msg)
{
    Walking::GetInstance()->X_MOVE_AMPLITUDE=(msg->linear.x);
    //TODO: Strafe disabled: Walking::GetInstance()->Y_MOVE_AMPLITUDE=(msg->linear.y);
    Walking::GetInstance()->A_MOVE_AMPLITUDE=(msg->angular.z);

    //ROS_INFO( "X: %0.1fmm/step Commanded", msg->linear.x );
    //ROS_INFO( "A: %0.1fdegrees/period Commanded", msg->angular.z );

    //ROS_INFO( "Walking GetBodySwingZ: %0.2f", Walking::GetInstance()->GetBodySwingZ() );
    //ROS_INFO("Walking: period %f x %f y %f a %f",Walking::GetInstance()->PERIOD_TIME,Walking::GetInstance()->X_MOVE_AMPLITUDE,Walking::GetInstance()->Y_MOVE_AMPLITUDE,Walking::GetInstance()->A_MOVE_AMPLITUDE);
}

void RobotHardwareInterface::cmdWalkingROSUnits(const geometry_msgs::Twist::ConstPtr& msg)
{
    Walking::GetInstance()->X_MOVE_AMPLITUDE=(msg->linear.x*1000.0);
    Walking::GetInstance()->Y_MOVE_AMPLITUDE=(msg->linear.y*1000.0);
    Walking::GetInstance()->A_MOVE_AMPLITUDE=((msg->angular.z*57.2958));

    //ROS_INFO( "X: %0.1fmm/step Commanded", msg->linear.x*1000.0 );
   // ROS_INFO( "A: %0.1fdegrees/period Commanded", msg->angular.z*57.2958 );
}

void RobotHardwareInterface::enableWalking(std_msgs::Int32ConstPtr enable)
{
    if( !Walking::GetInstance()->IsRunning() && enable->data > 0 )
    {
        setBlockWrite(true);
        if ( enable->data == 2 ) //Enable walk gait with lower body only
        {
            Walking::GetInstance()->m_Joint.SetEnableLowerBody(true, true);
        }
        else
        {
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
        }
        Walking::GetInstance()->Start();

        msg_is_walking_.data = true;
        is_walking_pub_.publish( msg_is_walking_ );
    }
    else if(Walking::GetInstance()->IsRunning() && !enable->data)
    {
        Walking::GetInstance()->Stop();
        while(Walking::GetInstance()->IsRunning())
        {
            ros::Duration(0.008).sleep();
        }

        setBlockWrite(false);
        if ( enable->data == 2 )
        {
            Walking::GetInstance()->m_Joint.SetEnableLowerBody(false, true);
        }
        else
        {
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(false, true);    
        }

        restoreROSControlConfig();
    }
}

//TODO: NOTE: enable,disable Walking added for navigation stack
void RobotHardwareInterface::enableWalking()
{
    if(!Walking::GetInstance()->IsRunning())
    {
        setBlockWrite(true);
        //Walking::GetInstance()->m_Joint.SetEnableLowerBody(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
        Walking::GetInstance()->Start();

        msg_is_walking_.data = true;
        is_walking_pub_.publish( msg_is_walking_ );
    }
}
void RobotHardwareInterface::disableWalking()
{
    if(Walking::GetInstance()->IsRunning())
    {
        Walking::GetInstance()->Stop();
        while(Walking::GetInstance()->IsRunning())
        {
            //usleep(8*1000);
            ros::Duration(0.008).sleep();
        }

        setBlockWrite(false);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(false, true);

        restoreROSControlConfig();
    }
}

bool RobotHardwareInterface::checkIsWalking(void)
{
    return Walking::GetInstance()->IsRunning();
}

double RobotHardwareInterface::getPeriodTime(void)
{
    return Walking::GetInstance()->PERIOD_TIME;
}

void RobotHardwareInterface::checkFall()
{
    //NOTE: Reporting fallen status every iteration regardless of walking status (safety concern)
    if ( MotionStatus::FALLEN != 0 )
    {
        msg_is_fallen_.data = true;
        is_fallen_pub_.publish( msg_is_fallen_ );
    }
    else if ( MotionStatus::FALLEN == 0 && msg_is_fallen_.data == true  )
    {
        msg_is_fallen_.data = false;
        is_fallen_pub_.publish( msg_is_fallen_ );
    }

    if(Walking::GetInstance()->IsRunning() && MotionStatus::FALLEN != 0)
    {
        Walking::GetInstance()->Stop();

        if(1 == MotionStatus::FALLEN)
        {
            ROS_WARN("Forward Fall! Getting up...");
            std_msgs::Int32 i;
            i.data=10; //TODO: Configurable action page for getting up
            startAction(i);
        }
        else
        {
            ROS_WARN("Backwards Fall! Getting up...");
            std_msgs::Int32 i;
            i.data=11; //TODO: Configurable action page for getting up
            startAction(i);
        }
    }
    if(print_check_fall_debug_info_)
        if(Walking::GetInstance()->IsRunning() || MotionStatus::FALLEN != 0)
            ROS_INFO("checkFall: walking %i fallen %i fb_acc %i rl_acc %i fb_gyro %0.4f rl_gyro %0.4f", Walking::GetInstance()->IsRunning(), MotionStatus::FALLEN,MotionStatus::FB_ACCEL,MotionStatus::RL_ACCEL,MotionStatus::FB_GYRO,MotionStatus::RL_GYRO);
}

void RobotHardwareInterface::restoreROSControlConfig(void)
{
    setPIDGains(1,m_ros_control_config_.RShoulderPitch_position_controller_p_gain, m_ros_control_config_.RShoulderPitch_position_controller_i_gain, m_ros_control_config_.RShoulderPitch_position_controller_d_gain);
    setPIDGains(2,m_ros_control_config_.LShoulderPitch_position_controller_p_gain, m_ros_control_config_.LShoulderPitch_position_controller_i_gain, m_ros_control_config_.LShoulderPitch_position_controller_d_gain);
    setPIDGains(3,m_ros_control_config_.RShoulderRoll_position_controller_p_gain, m_ros_control_config_.RShoulderRoll_position_controller_i_gain, m_ros_control_config_.RShoulderRoll_position_controller_d_gain);
    setPIDGains(4,m_ros_control_config_.LShoulderRoll_position_controller_p_gain, m_ros_control_config_.LShoulderRoll_position_controller_i_gain, m_ros_control_config_.LShoulderRoll_position_controller_d_gain);
    setPIDGains(5,m_ros_control_config_.RElbowPitch_position_controller_p_gain, m_ros_control_config_.RElbowPitch_position_controller_i_gain, m_ros_control_config_.RElbowPitch_position_controller_d_gain);
    setPIDGains(6,m_ros_control_config_.LElbowPitch_position_controller_p_gain, m_ros_control_config_.LElbowPitch_position_controller_i_gain, m_ros_control_config_.LElbowPitch_position_controller_d_gain);
    setPIDGains(7,m_ros_control_config_.RHipYaw_position_controller_p_gain, m_ros_control_config_.RHipYaw_position_controller_i_gain, m_ros_control_config_.RHipYaw_position_controller_d_gain);
    setPIDGains(8,m_ros_control_config_.LHipYaw_position_controller_p_gain, m_ros_control_config_.LHipYaw_position_controller_i_gain, m_ros_control_config_.LHipYaw_position_controller_d_gain);
    setPIDGains(9,m_ros_control_config_.RHipRoll_position_controller_p_gain, m_ros_control_config_.RHipRoll_position_controller_i_gain, m_ros_control_config_.RHipRoll_position_controller_d_gain);
    setPIDGains(10,m_ros_control_config_.LHipRoll_position_controller_p_gain, m_ros_control_config_.LHipRoll_position_controller_i_gain, m_ros_control_config_.LHipRoll_position_controller_d_gain);
    setPIDGains(11,m_ros_control_config_.RHipPitch_position_controller_p_gain, m_ros_control_config_.RHipPitch_position_controller_i_gain, m_ros_control_config_.RHipPitch_position_controller_d_gain);
    setPIDGains(12,m_ros_control_config_.LHipPitch_position_controller_p_gain, m_ros_control_config_.LHipPitch_position_controller_i_gain, m_ros_control_config_.LHipPitch_position_controller_d_gain);
    setPIDGains(13,m_ros_control_config_.RKneePitch_position_controller_p_gain, m_ros_control_config_.RKneePitch_position_controller_i_gain, m_ros_control_config_.RKneePitch_position_controller_d_gain);
    setPIDGains(14,m_ros_control_config_.LKneePitch_position_controller_p_gain, m_ros_control_config_.LKneePitch_position_controller_i_gain, m_ros_control_config_.LKneePitch_position_controller_d_gain);
    setPIDGains(15,m_ros_control_config_.RAnklePitch_position_controller_p_gain, m_ros_control_config_.RAnklePitch_position_controller_i_gain, m_ros_control_config_.RAnklePitch_position_controller_d_gain);
    setPIDGains(16,m_ros_control_config_.LAnklePitch_position_controller_p_gain, m_ros_control_config_.LAnklePitch_position_controller_i_gain, m_ros_control_config_.LAnklePitch_position_controller_d_gain);
    setPIDGains(17,m_ros_control_config_.RAnkleRoll_position_controller_p_gain, m_ros_control_config_.RAnkleRoll_position_controller_i_gain, m_ros_control_config_.RAnkleRoll_position_controller_d_gain);
    setPIDGains(18,m_ros_control_config_.LAnkleRoll_position_controller_p_gain, m_ros_control_config_.LAnkleRoll_position_controller_i_gain, m_ros_control_config_.LAnkleRoll_position_controller_d_gain);
    setPIDGains(19,m_ros_control_config_.HeadYaw_position_controller_p_gain, m_ros_control_config_.HeadYaw_position_controller_i_gain, m_ros_control_config_.HeadYaw_position_controller_d_gain);
    setPIDGains(20,m_ros_control_config_.HeadPitch_position_controller_p_gain, m_ros_control_config_.HeadPitch_position_controller_i_gain, m_ros_control_config_.HeadPitch_position_controller_d_gain);
}

void RobotHardwareInterface::startAction(std_msgs::Int32 action_index)
{
    setBlockWrite(true);
    //enable MotionManager if necessary
    if(!MotionManager::GetInstance()->GetEnable())
        MotionManager::GetInstance()->SetEnable(true);
    // stop walking if necessary
    if(Walking::GetInstance()->IsRunning())
    {
        Walking::GetInstance()->Stop();
        while(Walking::GetInstance()->IsRunning())
        {
            //usleep(8*1000);
            ros::Duration(0.008).sleep();
        }

        setBlockWrite(false);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(false, true);

        msg_is_walking_.data = false;
        is_walking_pub_.publish( msg_is_walking_ );
    }
    // stop any previous action
    if(Action::GetInstance()->IsRunning())
    {
        Action::GetInstance()->Stop();
        while(Action::GetInstance()->IsRunning())
        {
            //usleep(8*1000);
            ros::Duration(0.008).sleep();
        }
    }

    // enable all the body servos to the action module
//TODO: This is just for show.. Do not commit    
    if ( (int)action_index.data == 21 /*new_wave*/ )
    {
        Action::GetInstance()->m_Joint.SetEnableLeftArmOnly(true,true);
    }
    else
    {
        Action::GetInstance()->m_Joint.SetEnableBody(true,true);
    }
    
    if(!Action::GetInstance()->Start((int)action_index.data))
    {
        ROS_ERROR("RobotHardwareInterface: Could not start Action %i",(int)action_index.data);
    }

    while(Action::GetInstance()->IsRunning())
    {
        //usleep(8*1000);
        ros::Duration(0.008).sleep();
    }

//TODO: This is just for show.. Do not commit    
if ( (int)action_index.data != 21 /*new_wave*/ )
{
    MotionManager::GetInstance()->Reinitialize();
    for (unsigned int joint_index = 1; joint_index < JointData::NUMBER_OF_JOINTS; joint_index++)
    {
        int id_index = joint_index-1;
        double new_pos = angles::from_degrees(MotionStatus::m_CurrentJoints.GetAngle(joint_index)+ros_joint_offsets[joint_index]);
        pos_[id_index]= new_pos;
    }
}

    setBlockWrite(false);

    Action::GetInstance()->m_Joint.SetEnableBody(false,true);

    MotionStatus::m_CurrentJoints.SetEnableBody(true,true);

//ROS_ERROR( "hardware interface played action and is attempting to restore ros control config" );
    restoreROSControlConfig();
}

void RobotHardwareInterface::setBlockWrite(bool block)
{
    if(block)
    {
        block_write_ = true;
        ROS_INFO("Blocked Write");
    }
    else
    {  
        std::vector<ros::Publisher> joint_publisher;
        ros::NodeHandle n;

        std_msgs::Float64 angle_msg;

        ros::Publisher j00_pub = n.advertise<std_msgs::Float64>("/hros5/RShoulderPitch_position_controller/command", 100);
        ros::Publisher j01_pub = n.advertise<std_msgs::Float64>("/hros5/LShoulderPitch_position_controller/command", 100);
        ros::Publisher j02_pub = n.advertise<std_msgs::Float64>("/hros5/RShoulderRoll_position_controller/command", 100);
        ros::Publisher j03_pub = n.advertise<std_msgs::Float64>("/hros5/LShoulderRoll_position_controller/command", 100);
        ros::Publisher j04_pub = n.advertise<std_msgs::Float64>("/hros5/RElbowPitch_position_controller/command", 100);
        ros::Publisher j05_pub = n.advertise<std_msgs::Float64>("/hros5/LElbowPitch_position_controller/command", 100);
        ros::Publisher j06_pub = n.advertise<std_msgs::Float64>("/hros5/RHipYaw_position_controller/command", 100);
        ros::Publisher j07_pub = n.advertise<std_msgs::Float64>("/hros5/LHipYaw_position_controller/command", 100);
        ros::Publisher j08_pub = n.advertise<std_msgs::Float64>("/hros5/RHipRoll_position_controller/command", 100);
        ros::Publisher j09_pub = n.advertise<std_msgs::Float64>("/hros5/LHipRoll_position_controller/command", 100);
        ros::Publisher j10_pub = n.advertise<std_msgs::Float64>("/hros5/RHipPitch_position_controller/command", 100);
        ros::Publisher j11_pub = n.advertise<std_msgs::Float64>("/hros5/LHipPitch_position_controller/command", 100);
        ros::Publisher j12_pub = n.advertise<std_msgs::Float64>("/hros5/RKneePitch_position_controller/command", 100);
        ros::Publisher j13_pub = n.advertise<std_msgs::Float64>("/hros5/LKneePitch_position_controller/command", 100);
        ros::Publisher j14_pub = n.advertise<std_msgs::Float64>("/hros5/RAnklePitch_position_controller/command", 100);
        ros::Publisher j15_pub = n.advertise<std_msgs::Float64>("/hros5/LAnklePitch_position_controller/command", 100);
        ros::Publisher j16_pub = n.advertise<std_msgs::Float64>("/hros5/RAnkleRoll_position_controller/command", 100);
        ros::Publisher j17_pub = n.advertise<std_msgs::Float64>("/hros5/LAnkleRoll_position_controller/command", 100);
        ros::Publisher j18_pub = n.advertise<std_msgs::Float64>("/hros5/HeadYaw_position_controller/command", 100);
        ros::Publisher j19_pub = n.advertise<std_msgs::Float64>("/hros5/HeadPitch_position_controller/command", 100);

        //usleep( 100000 ); //TODO: Testing delay after advertisting. This was noticed to be necessary in part of the simple_rover_node's creation
        ros::Duration(0.1).sleep();

        joint_publisher.push_back(j00_pub);
        joint_publisher.push_back(j01_pub);
        joint_publisher.push_back(j02_pub);
        joint_publisher.push_back(j03_pub);
        joint_publisher.push_back(j04_pub);
        joint_publisher.push_back(j05_pub);
        joint_publisher.push_back(j06_pub);
        joint_publisher.push_back(j07_pub);
        joint_publisher.push_back(j08_pub);
        joint_publisher.push_back(j09_pub);
        joint_publisher.push_back(j10_pub);
        joint_publisher.push_back(j11_pub);
        joint_publisher.push_back(j12_pub);
        joint_publisher.push_back(j13_pub);
        joint_publisher.push_back(j14_pub);
        joint_publisher.push_back(j15_pub);
        joint_publisher.push_back(j16_pub);
        joint_publisher.push_back(j17_pub);
        joint_publisher.push_back(j18_pub);
        joint_publisher.push_back(j19_pub);

        for(unsigned int i = 0; i<joint_publisher.size(); ++i)
        {
            angle_msg.data = pos_[i];
            joint_publisher[i].publish(angle_msg);
        }

        block_write_ = false;
        ROS_INFO("Unblocked Write");
    }
}
}
