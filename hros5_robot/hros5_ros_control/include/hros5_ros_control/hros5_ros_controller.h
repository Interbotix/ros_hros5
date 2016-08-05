#ifndef hros5_ROS_CONTROLLER_H_
#define hros5_ROS_CONTROLLER_H_

// Robotis Framework
#include <LinuxDARwIn.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

// ROS Control
#include <controller_manager/controller_manager.h>
#include <hros5_ros_control/hros5_hardware_interface.h>

// IMU
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <hros5_ros_control/hros5_ros_controlConfig.h>

namespace hros5
{
class RobotisOPRosControllerNode
{
public:
    RobotisOPRosControllerNode();
    ~RobotisOPRosControllerNode();

    void update(ros::Time time, ros::Duration period);
    void dynamicReconfigureCb(hros5_ros_control::hros5_ros_controlConfig &config, uint32_t level);

protected:
    geometry_msgs::Twist gait_vel_;
    geometry_msgs::Point position_;
    
    bool ignore_teleop_;
    
    ros::Time current_time, last_time;
double pose_x_; // pose/odometry
double pose_y_; // pose/odometry
double pose_th_; // pose/odometry
    void publishOdometry( const geometry_msgs::Twist &gait_vel );
geometry_msgs::Quaternion imu_orientation_;
    void setTorqueOn(std_msgs::BoolConstPtr enable);

    // Callbacks
    void standSitCb( std_msgs::BoolConstPtr p_standing );
    void enableWalkCb(std_msgs::Int32ConstPtr enable);
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg);
    void cmdNavVelCb(const geometry_msgs::Twist::ConstPtr& msg);
    void startActionCb(std_msgs::Int32 action);

    // Subscriber
    ros::Subscriber torque_on_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber navigation_cmd_vel_sub_;
    ros::Subscriber start_action_sub_;
    ros::Subscriber enable_walk_sub_;
    ros::Subscriber stand_sit_sub_;

    // Publisher
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster;

    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

};
}

#endif

