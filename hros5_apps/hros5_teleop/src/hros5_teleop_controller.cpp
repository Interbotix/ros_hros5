//based on teleop_base_keyboard by Willow Garage, Inc.
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


class HROS5TeleopController
{
public:
    HROS5TeleopController();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void jointStatesCb(const sensor_msgs::JointState& msg);

    ros::NodeHandle nh_;

    ros::Time button_debounce_time_;

    int button_standup_, button_sitdown_, button_head_look_, button_toggle_walking_;
    int axis_linear_x_, axis_linear_y_, axis_angular_, axis_head_tilt;
    double l_scale_, a_scale_, panh_scale_, tilt_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    std_msgs::Bool enable_walking_msg;
    std_msgs::Bool sit_stand_msg;

    ros::Publisher tilt_pub_;
    ros::Publisher panh_pub_;
    ros::Publisher actionh_pub_;
    ros::Publisher enable_walking_pub_;
    ros::Publisher sit_stand_pub_;

    ros::Subscriber joint_states_sub_;

    bool walking_enabled, standing_state;
    float pan, tilt;
};


HROS5TeleopController::HROS5TeleopController():
    //NOTE: These defaults apply to a DS4 controller
    axis_linear_x_(1),
    axis_linear_y_(0),
    axis_angular_(2),
    axis_head_tilt(5),
    button_standup_(3),
    button_sitdown_(1),
    button_head_look_(4),
    button_toggle_walking_(5)
{

    nh_.param("axis_linear_forwards", axis_linear_x_, axis_linear_x_);
    nh_.param("axis_linear_sidewards", axis_linear_y_, axis_linear_y_);
    nh_.param("axis_angular", axis_angular_, axis_angular_);
    nh_.param("axis_head_tilt", axis_head_tilt, axis_head_tilt);
    
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("scale_pan", panh_scale_, panh_scale_);
    nh_.param("scale_tilt", tilt_scale_, tilt_scale_);
    walking_enabled = false;
    pan = 0.0;
    tilt = 0.0;

    joint_states_sub_ = nh_.subscribe("/hros5/joint_states", 100, &HROS5TeleopController::jointStatesCb, this);

    tilt_pub_ = nh_.advertise<std_msgs::Float64>("/hros5/HeadPitch_position_controller/command", 2);
    panh_pub_ = nh_.advertise<std_msgs::Float64>("/hros5/HeadYaw_position_controller/command", 2);
    actionh_pub_ = nh_.advertise<std_msgs::Int32>("/hros5/start_action", 2);
    enable_walking_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/enable_walking", 2);
    sit_stand_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/standing_sitting", 2);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("hros5/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &HROS5TeleopController::joyCallback, this);
}

void HROS5TeleopController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //NOTE: Some controllers with motion sensing will trigger callback on movement negating the autorepeat_rate of 0, debounce buttons
    if ( ros::Time::now() > button_debounce_time_ + ros::Duration(0.5) )
    {
        if ( joy->buttons[button_standup_] == 1 )
        { 
            button_debounce_time_ = ros::Time::now();
            ROS_INFO("Standing up");
            sit_stand_msg.data = 1;
            sit_stand_pub_.publish(sit_stand_msg);
            standing_state = true;
            walking_enabled = false; //If an action page is played walking will stop in RobotHardwareInterface::startAction
        }
        else if ( joy->buttons[button_sitdown_] == 1 )
        { 
            button_debounce_time_ = ros::Time::now();
            ROS_INFO("Sitting down");
            if ( walking_enabled == true )
            {
                ROS_WARN("Disabling walking before sitting");
                /* NOT REQUIRED BY DESIGN //See RobotHardwareInterface::startAction
                enable_walking_msg.data = 0;
                enable_walking_pub_.publish(enable_walking_msg);
                */
                walking_enabled = false;
            }
            sit_stand_msg.data = 0;
            sit_stand_pub_.publish(sit_stand_msg);
            standing_state = false;
        }

        if ( joy->buttons[button_toggle_walking_] == 1 )
        { 
            button_debounce_time_ = ros::Time::now();
            if ( walking_enabled == false )
            {
                if ( standing_state == true )
                {
                    ROS_INFO("Enable walking");
                    enable_walking_msg.data = 1;
                    enable_walking_pub_.publish(enable_walking_msg);
                    walking_enabled = true;
                }
                else
                {
                    ROS_WARN("Enable walking requires standing");
                }
            }
            else
            {
                ROS_INFO("Disable walking");
                enable_walking_msg.data = 0;
                enable_walking_pub_.publish(enable_walking_msg);
                walking_enabled = false;
            }
        }
    }

    if ( /*TODO: walking_enabled ==*/ true )
    {
        //TODO: Use twist stamped so controller can determine if a timeout occurs
        geometry_msgs::Twist vel;
        vel.angular.z = a_scale_*joy->axes[axis_angular_];
        vel.linear.x = l_scale_*joy->axes[axis_linear_x_];
        vel.linear.y = l_scale_*joy->axes[axis_linear_y_];
        vel_pub_.publish(vel);
    }
    
    if ( joy->buttons[button_head_look_] == 1 )
    {
        std_msgs::Float64 angle_msg;
        
        if ( tilt < 0.57 && joy->axes[axis_head_tilt] > 0.0 ) //TODO: Configurable limit?
        {
            angle_msg.data = tilt+tilt_scale_*joy->axes[axis_head_tilt];
            tilt_pub_.publish(angle_msg);
        }
        else if ( tilt > -0.35 && joy->axes[axis_head_tilt] < 0.0) //TODO: Configurable limit?
        {
            angle_msg.data = tilt+tilt_scale_*joy->axes[axis_head_tilt];
            tilt_pub_.publish(angle_msg);
        }
        
        if ( pan < 0.6 && joy->axes[axis_angular_] > 0.0 || pan > -0.6 && joy->axes[axis_angular_] < 0.0) //TODO: Configurable limit?
        {
            angle_msg.data = pan+panh_scale_*joy->axes[axis_angular_];
            panh_pub_.publish(angle_msg);
        }
    }
}


void HROS5TeleopController::jointStatesCb(const sensor_msgs::JointState& msg)
{
    if ( msg.name.at(1) == "HeadYaw" )    pan = msg.position.at(1);
    else ROS_ERROR_STREAM( "JointStates msg.at(1) is not 'HeadYaw': Cannot update HeadYaw with value from: " << msg.name.at(1) );

    if ( msg.name.at(0) == "HeadPitch")    tilt = msg.position.at(0);
    else ROS_ERROR_STREAM( "JointStates msg.at(0) is not 'HeadPitch': Cannot update HeadPitch with value from: " << msg.name.at(0) );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hros5_teleop_controller_node");
    HROS5TeleopController hros5_teleop_controller_node;

    ros::spin();
}
