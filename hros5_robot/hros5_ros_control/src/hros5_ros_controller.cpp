#include <hros5_ros_control/hros5_ros_controller.h>
#include <LinuxMotionTimer.h>
#include <sensor_msgs/Imu.h>

namespace hros5
{
RobotisOPRosControllerNode::RobotisOPRosControllerNode()
{
    ros::NodeHandle nh;

    // Initialize ros control
    controller_manager_.reset(new controller_manager::ControllerManager(RobotHardwareInterface::Instance().get(), nh));

    // Subscribe topics
    torque_on_sub_ = nh.subscribe("torque_on", 1, &RobotisOPRosControllerNode::setTorqueOn, this);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 25, &RobotisOPRosControllerNode::cmdVelCb, this);
    navigation_cmd_vel_sub_ = nh.subscribe("nav_cmd_vel", 25, &RobotisOPRosControllerNode::cmdNavVelCb, this);
    start_action_sub_ = nh.subscribe("start_action", 1, &RobotisOPRosControllerNode::startActionCb, this);
    enable_walk_sub_ = nh.subscribe("enable_walking", 2, &RobotisOPRosControllerNode::enableWalkCb, this);
    stand_sit_sub_ = nh.subscribe("standing_sitting", 2, &RobotisOPRosControllerNode::standSitCb, this);
    load_gait_ini_sub_ = nh.subscribe("walk_gait_load_ini", 2, &RobotisOPRosControllerNode::loadGaitIniCb, this);

    //Publish topics
    odom_pub_ = nh.advertise<nav_msgs::Odometry>( "odom", 50 );
    
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //TODO: set height when action is == darwin sit or stand (9,15)?
    position_.z = 0.24; //Torso height 240mm in sitting position
    position_.z = 0.24 + 0.14; //TODO: Calculate height.. //This is about standing position

ignore_teleop_ = false;
    ROS_INFO("Initialization of ros controller completed!");
}

RobotisOPRosControllerNode::~RobotisOPRosControllerNode()
{
    ROS_INFO("Cleaning up RobotisOPRosControllerNode...");
}

void RobotisOPRosControllerNode::setTorqueOn(std_msgs::BoolConstPtr enable)
{
    RobotHardwareInterface::Instance()->setTorqueOn(enable->data);
}

void RobotisOPRosControllerNode::cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    if ( ignore_teleop_ == true )
    {
        ROS_WARN_ONCE( "Joystick teleop velocities are ignored during navigation. This is your last warning.");
        return; //TODO: Ignoring Teleop velocities!!!!!!!
    }

    RobotHardwareInterface::Instance()->cmdWalking(msg);

    double period_time = RobotHardwareInterface::Instance()->getPeriodTime();

    gait_vel_.linear.x = (msg->linear.x/1000.0)/(period_time/1000.0)*4.0;
    //gait_vel_.linear.y = (msg->linear.y/1000.0)/(period_time/1000.0)*4.0;
    gait_vel_.angular.z = (msg->angular.z/57.2958)/(period_time/1000.0)*0.8;

    //TODO: This is a poor way of shifting the robot left and right as the darwin gait turns
    //if ( msg->angular.z > 0.0 ) //turning clockwise.. slight right side step
    gait_vel_.linear.y = (msg->angular.z/1000.0)/(period_time/1000.0);
    if ( msg->angular.z > 0.0 )
        gait_vel_.linear.x += (msg->angular.z/1000.0)/(period_time/1000.0);
    else if ( msg->angular.z < 0.0 )
        gait_vel_.linear.x -= (msg->angular.z/1000.0)/(period_time/1000.0);
}

void RobotisOPRosControllerNode::cmdNavVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    //TODO: FIX UNITS (ROS Compliant)
    //TODO: Not ready to move robot yet..
    //TODO: Warning.. we better be STANDING UP!!!
    if ( !RobotHardwareInterface::Instance()->checkIsWalking() )
    {
        ignore_teleop_ = true;
        ROS_WARN( "Nav starting walk gait" );
        RobotHardwareInterface::Instance()->enableWalking();
    }

    RobotHardwareInterface::Instance()->cmdWalkingROSUnits(msg);

    double period_time = RobotHardwareInterface::Instance()->getPeriodTime();

    gait_vel_.linear.x = msg->linear.x/(period_time/1000.0)*4.0;
    gait_vel_.linear.y = msg->linear.y/(period_time/1000.0)*4.0;
    gait_vel_.angular.z = msg->angular.z/(period_time/1000.0);
    ROS_INFO_STREAM( "Nav Cmd Vel.. X: " << msg->linear.x << " Z: " << msg->angular.z << " Y: " << msg->linear.y );

    //TODO: This is a poor way of shifting the robot left and right as the darwin gait turns
    //if ( msg->angular.z > 0.0 ) //turning clockwise.. slight right side step
    gait_vel_.linear.y = ((msg->angular.z*57.2958)/1000.0)/(period_time/1000.0);
    if ( msg->angular.z > 0.0 )
        gait_vel_.linear.x += ((msg->angular.z*57.2958)/1000.0)/(period_time/1000.0);
    else if ( msg->angular.z < 0.0 )
        gait_vel_.linear.x -= ((msg->angular.z*57.2958)/1000.0)/(period_time/1000.0);

    if ( msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->angular.z == 0.0 )
    {
        ignore_teleop_ = false;
        if ( RobotHardwareInterface::Instance()->checkIsWalking() )
        {
            ROS_WARN( "Nav stopping walk gait" );
            RobotHardwareInterface::Instance()->disableWalking();
        }
    }
}

void RobotisOPRosControllerNode::startActionCb(std_msgs::Int32 action)
{
    RobotHardwareInterface::Instance()->startAction(action);
}

void RobotisOPRosControllerNode::enableWalkCb(std_msgs::Int32ConstPtr enable)
{
    RobotHardwareInterface::Instance()->enableWalking(enable);
}

void RobotisOPRosControllerNode::standSitCb( std_msgs::BoolConstPtr p_standing )
{
    std_msgs::Int32 action;
    
    if ( p_standing->data )
    {
        action.data = 9;
        RobotHardwareInterface::Instance()->startAction(action); //Walk Ready

        position_.z = 0.24 + 0.14; //TODO: Calculate height..
    }
    else
    {
        action.data = 15;
        RobotHardwareInterface::Instance()->startAction(action); //Sit down

        position_.z = 0.24;
    }
}

void RobotisOPRosControllerNode::loadGaitIniCb( std_msgs::StringConstPtr file_path )
{
    RobotHardwareInterface::Instance()->loadWalkGaitINI(file_path);
}

void RobotisOPRosControllerNode::dynamicReconfigureCb(hros5_ros_control::hros5_ros_controlConfig &config, uint32_t level)
{
    RobotHardwareInterface::Instance()->storeROSControlConfig( config );
    RobotHardwareInterface::Instance()->setPIDGains(1,config.RShoulderPitch_position_controller_p_gain, config.RShoulderPitch_position_controller_i_gain, config.RShoulderPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(2,config.LShoulderPitch_position_controller_p_gain, config.LShoulderPitch_position_controller_i_gain, config.LShoulderPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(3,config.RShoulderRoll_position_controller_p_gain, config.RShoulderRoll_position_controller_i_gain, config.RShoulderRoll_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(4,config.LShoulderRoll_position_controller_p_gain, config.LShoulderRoll_position_controller_i_gain, config.LShoulderRoll_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(5,config.RElbowPitch_position_controller_p_gain, config.RElbowPitch_position_controller_i_gain, config.RElbowPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(6,config.LElbowPitch_position_controller_p_gain, config.LElbowPitch_position_controller_i_gain, config.LElbowPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(7,config.RHipYaw_position_controller_p_gain, config.RHipYaw_position_controller_i_gain, config.RHipYaw_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(8,config.LHipYaw_position_controller_p_gain, config.LHipYaw_position_controller_i_gain, config.LHipYaw_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(9,config.RHipRoll_position_controller_p_gain, config.RHipRoll_position_controller_i_gain, config.RHipRoll_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(10,config.LHipRoll_position_controller_p_gain, config.LHipRoll_position_controller_i_gain, config.LHipRoll_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(11,config.RHipPitch_position_controller_p_gain, config.RHipPitch_position_controller_i_gain, config.RHipPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(12,config.LHipPitch_position_controller_p_gain, config.LHipPitch_position_controller_i_gain, config.LHipPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(13,config.RKneePitch_position_controller_p_gain, config.RKneePitch_position_controller_i_gain, config.RKneePitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(14,config.LKneePitch_position_controller_p_gain, config.LKneePitch_position_controller_i_gain, config.LKneePitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(15,config.RAnklePitch_position_controller_p_gain, config.RAnklePitch_position_controller_i_gain, config.RAnklePitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(16,config.LAnklePitch_position_controller_p_gain, config.LAnklePitch_position_controller_i_gain, config.LAnklePitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(17,config.RAnkleRoll_position_controller_p_gain, config.RAnkleRoll_position_controller_i_gain, config.RAnkleRoll_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(18,config.LAnkleRoll_position_controller_p_gain, config.LAnkleRoll_position_controller_i_gain, config.LAnkleRoll_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(19,config.HeadYaw_position_controller_p_gain, config.HeadYaw_position_controller_i_gain, config.HeadYaw_position_controller_d_gain);
    RobotHardwareInterface::Instance()->setPIDGains(20,config.HeadPitch_position_controller_p_gain, config.HeadPitch_position_controller_i_gain, config.HeadPitch_position_controller_d_gain);
    RobotHardwareInterface::Instance()->print_check_fall_debug_info_ = config.print_check_fall_debug_info;
}

void RobotisOPRosControllerNode::update(ros::Time time, ros::Duration period)
{
    // ros control update cycle
    RobotHardwareInterface::Instance()->read(time, period);
    controller_manager_->update(time, period);
    RobotHardwareInterface::Instance()->write(time, period);

    //NOTE: If we are not walking be sure to ignore controller/twist input for odom
    if ( !RobotHardwareInterface::Instance()->checkIsWalking() ) 
    { 
        gait_vel_.linear.x = 0.0;
        gait_vel_.linear.y = 0.0;
        gait_vel_.angular.z = 0.0;
    }

    publishOdometry( gait_vel_ );

    RobotHardwareInterface::Instance()->checkFall();
}

//==============================================================================
// Odometry Publisher
//==============================================================================
void RobotisOPRosControllerNode::publishOdometry( const geometry_msgs::Twist &gait_vel )
{
    current_time = ros::Time::now();

    // compute odometry in a typical way given the velocities of the robot
    double dt = ( current_time - last_time ).toSec();

    double vth = gait_vel.angular.z;
    double delta_th = vth * dt;
    pose_th_ += delta_th;

    double vx = gait_vel.linear.x;
    double vy = gait_vel.linear.y;
    double delta_x = ( vx * cos( pose_th_ ) - vy * sin( pose_th_ ) ) * dt;
    double delta_y = ( vx * sin( pose_th_ ) + vy * cos( pose_th_ ) ) * dt;
    pose_x_ += delta_x;
    pose_y_ += delta_y;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( pose_th_ );

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose_x_;
    odom_trans.transform.translation.y = pose_y_;
    odom_trans.transform.translation.z = position_.z;
    odom_trans.transform.rotation = odom_quat;

    // Uncomment odom_broadcaster to send the transform. Only used if debugging calculated odometry.
    //odom_broadcaster.sendTransform( odom_trans );

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // set the position
    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    odom.pose.pose.position.z = position_.z;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = 0.00001;  // x
    odom.pose.covariance[7] = 0.00001;  // y
    odom.pose.covariance[14] = 0.00001; // z
    odom.pose.covariance[21] = 1000000000000.0; // rot x
    odom.pose.covariance[28] = 1000000000000.0; // rot y
    odom.pose.covariance[35] = 0.001; // rot z

    // set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = odom.pose.covariance; // needed?

    odom_pub_.publish( odom );

    last_time = current_time;
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hros5_ros_controller");

    ros::NodeHandle nh;
    double control_rate;
    nh.param("hros5_ros_controller/control_rate", control_rate, 125.0);

    hros5::RobotisOPRosControllerNode hros5_ros_controller_node;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);

    dynamic_reconfigure::Server<hros5_ros_control::hros5_ros_controlConfig> srv;
    dynamic_reconfigure::Server<hros5_ros_control::hros5_ros_controlConfig>::CallbackType cb;
    cb = boost::bind(&hros5::RobotisOPRosControllerNode::dynamicReconfigureCb, &hros5_ros_controller_node, _1, _2);
    srv.setCallback(cb);

    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        hros5_ros_controller_node.update(current_time, elapsed_time);
        last_time = current_time;
    }

    return 0;
}
