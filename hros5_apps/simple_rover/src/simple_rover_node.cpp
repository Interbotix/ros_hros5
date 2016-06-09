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
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define STATE_INIT 1
#define STATE_FORWARD 2
#define STATE_REVERSE 3
#define STATE_TURNING 4
#define STATE_FALLEN 5

class HROS5TeleopController
{
public:
    HROS5TeleopController();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void fallDetectionCallback(const std_msgs::Bool::ConstPtr& isFallen);
    
    ros::NodeHandle nh_;

    ros::Subscriber laser_sub_;
    ros::Subscriber fall_detection_sub_;

    ros::Publisher vel_pub_;
    ros::Publisher enable_walking_pub_;
    ros::Publisher sit_stand_pub_;

    std_msgs::Bool enable_walking_msg;
    std_msgs::Bool sit_stand_msg;

public:
    void publishVelocity( void );

    int current_state_;

    bool walking_enabled, standing_state, has_obstacle;

    geometry_msgs::Twist command_vel_;

    bool robot_has_fallen;

    void standRobotStartWalking( void );
};

HROS5TeleopController::HROS5TeleopController()
{
    //nh_.param("scale_angular", a_scale_, a_scale_);
    //nh_.param("scale_linear", l_scale_, l_scale_);
    robot_has_fallen = false;

    current_state_ = STATE_INIT;

    walking_enabled = false;
    standing_state = false;
    has_obstacle = false;

    command_vel_.angular.z = 0.0;
    command_vel_.linear.x = 0.0;
    command_vel_.linear.y = 0.0;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("hros5/cmd_vel", 1);
    enable_walking_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/enable_walking", 1);
    sit_stand_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/standing_sitting", 1);

    system( "espeak \"Starting simple_rover_node\"" );
    standRobotStartWalking();

    ROS_INFO("Subscribing to: /camera/scan" );
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/camera/scan", 1, &HROS5TeleopController::laserCallback, this);

    ROS_INFO("Subscribing to: /hros5/isFallen" );
    fall_detection_sub_ = nh_.subscribe<std_msgs::Bool>("/hros5/isFallen", 1, &HROS5TeleopController::fallDetectionCallback, this);

    current_state_ = STATE_FORWARD; //Start state machine
}


void HROS5TeleopController::standRobotStartWalking( void )
{
    sleep(1);
    { 
        ROS_INFO("Standing up");
        sit_stand_msg.data = 1;
        sit_stand_pub_.publish(sit_stand_msg);
        standing_state = true;
        walking_enabled = false; //If an action page is played walking will stop in RobotHardwareInterface::startAction
    }
    system( "espeak \"Ready to walk\"" );
    sleep( 4 );
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
}

void HROS5TeleopController::fallDetectionCallback(const std_msgs::Bool::ConstPtr& isFallen)
{
    if ( isFallen->data == true )
    {
        //ROS_WARN( "Fall reported. Stopping state machine." );
        robot_has_fallen = true;
    }
    else
    {
        robot_has_fallen = false;
    }
}

void HROS5TeleopController::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int c_foundCloseObject = 0;
    for ( int i=0; i<scan->ranges.size(); ++i )
    {
        if ( scan->ranges[i] < 0.95 )
        {
            ++c_foundCloseObject;
        }
    }
    //ROS_INFO_STREAM( "Scan points within threshold (" << c_foundCloseObject << ")" );

    if ( c_foundCloseObject > 35 )
    {
        has_obstacle = true;
        ROS_WARN( "HAS Obstable" );
    }
    else if ( has_obstacle == true )
    {
        ROS_WARN( "Obstable Clear" );
        has_obstacle = false;
    }

    /*
    if ( c_foundCloseObject > 50 && walking_enabled == true )
    {
        if ( !backing_up && !turning_left )
        {
            backing_up = true;
        }

        if ( backing_up )
        {

        }
        { //Disable system!
            walking_enabled = false;
            ROS_WARN_STREAM( "Obstable Detected: threshold (" << c_foundCloseObject << ")" );
            ROS_INFO("Disable walking");
            enable_walking_msg.data = 0;
            enable_walking_pub_.publish(enable_walking_msg);
            walking_enabled = false;
    //usleep( 100000 );
            system( "espeak \"Shit an obstacle. Stopping Everything\"" );
            ROS_INFO("Sitting down");
            sit_stand_msg.data = 0;
            sit_stand_pub_.publish(sit_stand_msg);
            standing_state = false;
        }
    }
    else if ( walking_enabled == true )
    {
        //Accelerate forward
        if ( command_vel_.linear.x < 25.0 ) command_vel_.linear.x += 0.5;
        command_vel_.angular.z = 0.0;
        command_vel_.linear.y = 0.0;
        vel_pub_.publish(command_vel_);
    }
    else
    {
        command_vel_.angular.z = 0.0;
        command_vel_.linear.x = 0.0;
        command_vel_.linear.y = 0.0;
        vel_pub_.publish(command_vel_);
    }
    */
    /*
    if ( false )
    { 
        ROS_INFO("Standing up");
        sit_stand_msg.data = 1;
        sit_stand_pub_.publish(sit_stand_msg);
        standing_state = true;
        walking_enabled = false; //If an action page is played walking will stop in RobotHardwareInterface::startAction
    }
    else if ( false )
    { 
        ROS_INFO("Sitting down");
        if ( walking_enabled == true )
        {
            ROS_WARN("Disabling walking before sitting");
            //NOT REQUIRED BY DESIGN //See RobotHardwareInterface::startAction
            //enable_walking_msg.data = 0;
            //enable_walking_pub_.publish(enable_walking_msg);
            //
            walking_enabled = false;
        }
        sit_stand_msg.data = 0;
        sit_stand_pub_.publish(sit_stand_msg);
        standing_state = false;
    }

    if ( false )
    { 
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

    if ( false )
    {
        //TODO: Use twist stamped so controller can determine if a timeout occurs
        geometry_msgs::Twist vel;
        //vel.angular.z = a_scale_*joy->axes[axis_angular_];
        //vel.linear.x = l_scale_*joy->axes[axis_linear_x_];
        //vel.linear.y = l_scale_*joy->axes[axis_linear_y_];
        vel_pub_.publish(vel);
    }
    */
}
void HROS5TeleopController::publishVelocity(void)
{
    vel_pub_.publish(command_vel_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_rover_node");
    HROS5TeleopController simple_rover_node;

    //ros::spin();
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    

    ros::Rate rate(25.0);
    ros::Time last_time = ros::Time::now();
    ros::Time turnStartTime;
    
    //Track time and duration of reverse state
    ros::Time reverseStartTime;
    ros::Duration reverseDuration;
    ros::Time reverseNowTime;
    
    //Throttle velocity publish from loop rate
    int publisherRate = 0; 
    
    //Track time since fall state was recovered
    bool stateFallen = false;
    ros::Time fallRecoveredStartTime;
    ros::Time fallRecoveredNowTime;
    ros::Duration fallRecoveredDuration;

    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        {
            //Check if fallen notification was received from controller node
            if ( simple_rover_node.robot_has_fallen == true ) 
            {
                if ( stateFallen == false )
                {
                    stateFallen = true; //Flag to watch for fall recovered state
                    system( "espeak \"Shit. I've fallen over\"" );

                    simple_rover_node.current_state_ = STATE_FALLEN;

                    simple_rover_node.command_vel_.linear.x = 0.0;
                    simple_rover_node.command_vel_.angular.z = 0.0;
                    simple_rover_node.command_vel_.linear.y = 0.0;
                    simple_rover_node.publishVelocity();
                }
                continue;
            }
            else if ( stateFallen == true )
            {
                system( "espeak \"That's better!\"" );
                fallRecoveredStartTime = ros::Time::now();
                stateFallen = false;
            }

            switch ( simple_rover_node.current_state_ )
            {
                case STATE_INIT:
                    simple_rover_node.command_vel_.linear.x = 0.0;
                    simple_rover_node.command_vel_.angular.z = 0.0;
                    simple_rover_node.command_vel_.linear.y = 0.0;
                break;
                case STATE_FORWARD:
                    if ( simple_rover_node.command_vel_.linear.x < 25.0 ) simple_rover_node.command_vel_.linear.x += 0.5;
                    simple_rover_node.command_vel_.angular.z = 0.0;
                    simple_rover_node.command_vel_.linear.y = 0.0;
                    if ( simple_rover_node.has_obstacle == true )
                    {
                        simple_rover_node.command_vel_.linear.x = 2.0;
                        system( "espeak \"backing up from object\"" );
                        ROS_INFO_STREAM( "Changing to state REVERSE, obstacle?" << simple_rover_node.has_obstacle );
                        simple_rover_node.current_state_ = STATE_REVERSE;
                        reverseStartTime = ros::Time::now();
                    }
                break;
                case STATE_REVERSE:
                    reverseNowTime = ros::Time::now();
                    reverseDuration = reverseNowTime - reverseStartTime;
                    simple_rover_node.command_vel_.angular.z = 0.0;
                    simple_rover_node.command_vel_.linear.y = 0.0;
                    if ( simple_rover_node.command_vel_.linear.x > -20.0 ) simple_rover_node.command_vel_.linear.x -= 0.5;
                    else if ( simple_rover_node.has_obstacle == false && reverseDuration.sec > 3 )
                    {
                        system( "espeak \"Turning away from object\"" );
                        ROS_INFO_STREAM( "Changing to state TURNING, obstacle?" << simple_rover_node.has_obstacle );
                        simple_rover_node.current_state_ = STATE_TURNING;
                        turnStartTime = ros::Time::now();
                    }
                break;
                case STATE_TURNING:
                    if ( simple_rover_node.command_vel_.linear.x > 0.0 ) 
                    {
                        simple_rover_node.command_vel_.linear.x -= 0.5;
                        break;
                    }
                    if ( simple_rover_node.command_vel_.linear.x < 0.0 )
                    {
                        simple_rover_node.command_vel_.linear.x += 0.5;
                        break;
                    }

                    if ( simple_rover_node.command_vel_.angular.z > -20.0 ) simple_rover_node.command_vel_.angular.z -= 0.5;
                    else
                    {
                        //Currently turning
                        ros::Time turnNowTime = ros::Time::now();
                        ros::Duration turningTime = turnNowTime - turnStartTime;
                        //ROS_INFO_STREAM( "Turn Duration( " << turningTime << " )" );

                        if ( turningTime.sec > 6 )
                        {
                            system( "espeak \"Moving forward\"" );
                            ROS_INFO( "Changing to state FORWARD" );
                            simple_rover_node.current_state_ = STATE_FORWARD;
                        }
                    }
                break;
                case STATE_FALLEN:
                    fallRecoveredNowTime = ros::Time::now();
                    fallRecoveredDuration = fallRecoveredNowTime - fallRecoveredStartTime;
                    if ( fallRecoveredDuration.sec > 5 )
                    {
                        simple_rover_node.current_state_ = STATE_INIT;
                        ROS_WARN( "State fallen is setting state init.. I've done as much as you've told me to do!" );
                        simple_rover_node.standRobotStartWalking();
                        simple_rover_node.current_state_ = STATE_FORWARD;
                    }
                break;
            }
            if ( ++publisherRate > 4 ) //Publish velocity at 5Hz. Walk gait will not change that quickly anway.
            {
                simple_rover_node.publishVelocity();
                publisherRate = 0;
            }


        }
        last_time = current_time;
    }

    ros::waitForShutdown();
    return 0;
}
