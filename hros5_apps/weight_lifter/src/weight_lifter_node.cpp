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
#include <std_msgs/String.h>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <opencv_apps/ContourArrayStamped.h>

#define STATE_INIT 1
#define STATE_APPROACH 2
#define STATE_LIFTBAR 3
#define STATE_FINISHLINE 4
#define STATE_COMPLETE 5
#define STATE_FALLEN 6

class HROS5WeightLifter
{
public:
    HROS5WeightLifter();

private:
    void contourCallback(const opencv_apps::ContourArrayStamped::ConstPtr& p_contours);
    void fallDetectionCallback(const std_msgs::Bool::ConstPtr& isFallen);
    
    ros::NodeHandle nh_;

    ros::Subscriber contour_sub_;
    ros::Subscriber fall_detection_sub_;

    ros::Publisher vel_pub_;
    ros::Publisher enable_walking_pub_;
    ros::Publisher sit_stand_pub_;
    ros::Publisher action_module_pub_;
    ros::Publisher walk_gait_ini_pub_;

    std_msgs::Int32 enable_walking_msg;
    std_msgs::Bool sit_stand_msg;

    std::string weighted_gait_config_;
public:
    std::string contour_topic;

    void publishVelocity( void );

    int current_state_;

    bool walking_enabled, standing_state, bar_detected;

    double computed_turn_angle;

    geometry_msgs::Twist command_vel_;

    bool robot_has_fallen;

    void standRobotStartWalking( void );

    void liftWeightBar( void );

    void walkToFinish( void );

    void stopWalking( void );
};

HROS5WeightLifter::HROS5WeightLifter()
{
    robot_has_fallen = false;

    current_state_ = STATE_INIT;

    walking_enabled = false;
    standing_state = false;
    bar_detected = false;

    command_vel_.angular.z = 0.0;
    command_vel_.linear.x = 0.0;
    command_vel_.linear.y = 0.0;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("hros5/cmd_vel", 1);
    enable_walking_pub_ = nh_.advertise<std_msgs::Int32>("/hros5/enable_walking", 1);
    sit_stand_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/standing_sitting", 1);
    action_module_pub_ = nh_.advertise<std_msgs::Int32>("/hros5/start_action", 1);
    walk_gait_ini_pub_ = nh_.advertise<std_msgs::String>("/hros5/walk_gait_load_ini", 1);

    weighted_gait_config_ = "NOTSET";
    if (!ros::param::has("~weighted_gait_config"))
    {
        ROS_ERROR_STREAM( "Config param 'weighted_gait_config' is not set" );
        ros::shutdown(); //TODO: not sure if this works
    }
    else
    {
        ros::param::get("~weighted_gait_config", weighted_gait_config_);
        ROS_INFO_STREAM( "Config param: " << weighted_gait_config_ );

        system( "espeak \"Starting weight_lifter_node\"" );
        standRobotStartWalking();

        ROS_INFO("Subscribing to: /contour_detector/contours" );
        contour_sub_ = nh_.subscribe<opencv_apps::ContourArrayStamped>("/contour_detector/contours", 1, &HROS5WeightLifter::contourCallback, this);

        ROS_INFO("Subscribing to: /hros5/isFallen" );
        fall_detection_sub_ = nh_.subscribe<std_msgs::Bool>("/hros5/isFallen", 1, &HROS5WeightLifter::fallDetectionCallback, this);

        current_state_ = STATE_APPROACH; //Start state machine
    }
}


void HROS5WeightLifter::standRobotStartWalking( void )
{
    sleep(1);
    /*
    { 
        ROS_INFO("Standing up");
        sit_stand_msg.data = 1;
        sit_stand_pub_.publish(sit_stand_msg);
        standing_state = true;
        walking_enabled = false; //If an action page is played walking will stop in RobotHardwareInterface::startAction
    }
    */
    standing_state = true;
    walking_enabled = false; //If an action page is played walking will stop in RobotHardwareInterface::startAction


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

void HROS5WeightLifter::stopWalking( void )
{
    enable_walking_msg.data = 0;
    enable_walking_pub_.publish(enable_walking_msg);
    walking_enabled = false;
}
void HROS5WeightLifter::liftWeightBar( void )
{
    std_msgs::Int32 c_msg;
    c_msg.data = 5; //Page 5 currently in the um7_darwin_work motion file
    action_module_pub_.publish(c_msg);

    //TODO: wait for action to complete
    sleep( 15 );
}

void HROS5WeightLifter::walkToFinish( void )
{
    ROS_INFO("loading alternate walk gait ini");
    std_msgs::String c_filepath;
    c_filepath.data = weighted_gait_config_;
    walk_gait_ini_pub_.publish( c_filepath );

    ROS_INFO("Enable walking without arms to finish");
    enable_walking_msg.data = 2; //enable walking level 2 is legs only
    enable_walking_pub_.publish(enable_walking_msg);
    walking_enabled = true;
}

void HROS5WeightLifter::fallDetectionCallback(const std_msgs::Bool::ConstPtr& isFallen)
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

void HROS5WeightLifter::contourCallback(const opencv_apps::ContourArrayStamped::ConstPtr& p_contours)
{
    double image_max_horizontal_pixels = 640.0; //TODO: not constant
    double image_max_verticle_pixels = 480.0; //TODO: not constant

    double average_top_horizonal_pos = 0.0;
    double average_bottom_horizonal_pos = 0.0;

    int contours_count = p_contours->contours.size();
    int contours_in_top = 0;
    int contours_in_bottom = 0;
    for( size_t i = 0; i < contours_count; i++ )
    {
        int points_count = p_contours->contours[i].points.size();
        int avg_top_count = 0; //number of points found in top half of image
        int avg_bottom_count = 0;
        double avg_x_top = 0.0; //average horizontal detection position in top half of image 
        double avg_x_bottom = 0.0;

        //Iterate contour points
        for ( size_t j = 0; j < points_count; j++ )
        {
            if ( p_contours->contours[i].points[j].y < image_max_verticle_pixels/2.0 )
            {
                //top half of image/view
                avg_x_top += p_contours->contours[i].points[j].x;
                ++avg_top_count;
            }
            else if ( p_contours->contours[i].points[j].y > image_max_verticle_pixels/2.0 )
            {
                //bottom half of image/view
                avg_x_bottom += p_contours->contours[i].points[j].x;
                ++avg_bottom_count;
            }
        }

        if ( avg_top_count > 0  )
        {
            avg_x_top /= avg_top_count;
            average_top_horizonal_pos += avg_x_top;
            ++contours_in_top;
        }
        if ( avg_bottom_count > 0 )
        {
            avg_x_bottom /= avg_bottom_count;
            average_bottom_horizonal_pos += avg_x_bottom;
            ++contours_in_bottom;
        }
    }
    average_top_horizonal_pos /= contours_in_top;
    average_top_horizonal_pos -= image_max_horizontal_pixels / 2.0; //zero center

    average_bottom_horizonal_pos /= contours_in_bottom;
    average_bottom_horizonal_pos -= image_max_horizontal_pixels / 2.0; //zero center

    ROS_INFO_STREAM( "contours: " << contours_count << " avg top horiz pos: " << average_top_horizonal_pos << " avg botm horiz pos: " << average_bottom_horizonal_pos );

    if ( contours_in_bottom > 2 )
    {
        bar_detected = true;
    }
}
void HROS5WeightLifter::publishVelocity(void)
{
    vel_pub_.publish(command_vel_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "weight_lifter_node");
    HROS5WeightLifter weight_lifter_node;

    //ros::spin();
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    ros::Rate rate(30.0);
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
            if ( weight_lifter_node.robot_has_fallen == true ) 
            {
                if ( stateFallen == false )
                {
                    stateFallen = true; //Flag to watch for fall recovered state
                    system( "espeak \"Shit. I've fallen over\"" );

                    weight_lifter_node.current_state_ = STATE_FALLEN;

                    weight_lifter_node.command_vel_.linear.x = 0.0;
                    weight_lifter_node.command_vel_.angular.z = 0.0;
                    weight_lifter_node.command_vel_.linear.y = 0.0;
                    weight_lifter_node.publishVelocity();
                }
                continue;
            }
            else if ( stateFallen == true )
            {
                system( "espeak \"That's better!\"" );
                fallRecoveredStartTime = ros::Time::now();
                stateFallen = false;
            }

            switch ( weight_lifter_node.current_state_ )
            {
                case STATE_INIT:
                    weight_lifter_node.command_vel_.linear.x = 0.0;
                    weight_lifter_node.command_vel_.angular.z = 0.0;
                    weight_lifter_node.command_vel_.linear.y = 0.0;
                break;
                case STATE_APPROACH:
                    if ( weight_lifter_node.command_vel_.linear.x < 5.0 ) weight_lifter_node.command_vel_.linear.x += 0.25;
                    weight_lifter_node.command_vel_.angular.z = 0.0;
                    weight_lifter_node.command_vel_.linear.y = 0.0;
                    
                    if ( weight_lifter_node.bar_detected == true )
                    {
                        weight_lifter_node.stopWalking();
                        weight_lifter_node.current_state_ = STATE_LIFTBAR;
                    }
                    
                break;
                case STATE_FALLEN:
                    fallRecoveredNowTime = ros::Time::now();
                    fallRecoveredDuration = fallRecoveredNowTime - fallRecoveredStartTime;
                    if ( fallRecoveredDuration.sec > 5 )
                    {
                        weight_lifter_node.current_state_ = STATE_COMPLETE;
                        ROS_WARN( "State fallen is setting state complete.. I've done as much as you've told me to do!" );
                    }
                break;
                case STATE_LIFTBAR:
                    system( "espeak \"grunt, grunt. pick this up, , , , , pick that up , , , , , when will it end?\" &" );
                    weight_lifter_node.liftWeightBar();
                    weight_lifter_node.current_state_ = STATE_FINISHLINE;
                    weight_lifter_node.walkToFinish();
                break;
                case STATE_FINISHLINE:
                    if ( weight_lifter_node.command_vel_.linear.x < 5.0 ) weight_lifter_node.command_vel_.linear.x += 0.25;
                    weight_lifter_node.command_vel_.angular.z = 0.0;
                    weight_lifter_node.command_vel_.linear.y = 0.0;
                break;
                case STATE_COMPLETE:

                break;
            }
            if ( ++publisherRate > 30 ) //Publish velocity at 1Hz. Walk gait will not change that quickly anyway.
            {
                weight_lifter_node.publishVelocity();
                publisherRate = 0;
            }


        }
        last_time = current_time;
    }

    ros::waitForShutdown();
    return 0;
}
