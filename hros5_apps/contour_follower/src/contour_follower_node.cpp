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
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <opencv_apps/ContourArrayStamped.h>

#define STATE_INIT 1
#define STATE_FORWARD 2
#define STATE_REVERSE 3
#define STATE_TURNING 4
#define STATE_FALLEN 5

class HROS5ContourFollower
{
public:
    HROS5ContourFollower();

private:
    void contourCallback(const opencv_apps::ContourArrayStamped::ConstPtr& p_contours);
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
    std::string contour_topic;

    void publishVelocity( void );

    int current_state_;

    bool walking_enabled, standing_state, curve_detected;

    double contour_turn_angle;

    geometry_msgs::Twist command_vel_;

    bool robot_has_fallen;

    void standRobotStartWalking( void );
};

HROS5ContourFollower::HROS5ContourFollower()
{
    robot_has_fallen = false;

    current_state_ = STATE_INIT;

    walking_enabled = false;
    standing_state = false;
    curve_detected = false;

    command_vel_.angular.z = 0.0;
    command_vel_.linear.x = 0.0;
    command_vel_.linear.y = 0.0;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("hros5/cmd_vel", 1);
    enable_walking_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/enable_walking", 1);
    sit_stand_pub_ = nh_.advertise<std_msgs::Bool>("/hros5/standing_sitting", 1);

    system( "espeak \"Starting contour_follower_node\"" );
    standRobotStartWalking();

    ROS_INFO("Subscribing to: /contour_detector/contours" );
    laser_sub_ = nh_.subscribe<opencv_apps::ContourArrayStamped>("/contour_detector/contours", 1, &HROS5ContourFollower::contourCallback, this);

    ROS_INFO("Subscribing to: /hros5/isFallen" );
    fall_detection_sub_ = nh_.subscribe<std_msgs::Bool>("/hros5/isFallen", 1, &HROS5ContourFollower::fallDetectionCallback, this);

    current_state_ = STATE_FORWARD; //Start state machine
}


void HROS5ContourFollower::standRobotStartWalking( void )
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

void HROS5ContourFollower::fallDetectionCallback(const std_msgs::Bool::ConstPtr& isFallen)
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

void HROS5ContourFollower::contourCallback(const opencv_apps::ContourArrayStamped::ConstPtr& p_contours)
{
    /*
    double average_contour_x = 0.0;
    double average_contour_y = 0.0;

    int contours_count = p_contours->contours.size();
    
    for( size_t i = 0; i < contours_count; i++ )
    {
        int points_size = p_contours->contours[i].points.size();
        double avg_x = 0.0;
        double avg_y = 0.0;

        for ( size_t j = 0; j < points_size; j++ )
        {
            avg_x += p_contours->contours[i].points[j].x;
            avg_y += p_contours->contours[i].points[j].y;
        }

        avg_x /= points_size;
        avg_y /= points_size;

        average_contour_x += avg_x;
        average_contour_y += avg_y;

    }
    average_contour_x /= contours_count;
    average_contour_y /= contours_count;

    ROS_ERROR_STREAM( "contours: " << contours_count << " avg x: " << average_contour_x << " avg y: " << average_contour_y );
    
    curve_detected = false;
    */

    /* Find angle of contour
    double average_contour_difference = 0.0;

    int contours_count = p_contours->contours.size();
    
    for( size_t i = 0; i < contours_count; i++ )
    {
        int points_size = p_contours->contours[i].points.size();
        int avg_top_count = 0;
        int avg_bottom_count = 0;
        double avg_y_top = 0.0;
        double avg_y_bottom = 0.0;

        double min_x = 9999.0;
        double max_x = 0.0;

        double min_y = 9999.9;
        double max_y = 0.0;

        //Determin min and max X and Y values of this contour
        for ( size_t j = 0; j < points_size; j++ )
        {
            if ( p_contours->contours[i].points[j].x < min_x ) min_x = p_contours->contours[i].points[j].x;
            if ( p_contours->contours[i].points[j].x > max_x ) max_x = p_contours->contours[i].points[j].x;

            if ( p_contours->contours[i].points[j].y < min_y ) min_y = p_contours->contours[i].points[j].y;
            if ( p_contours->contours[i].points[j].y > max_y ) max_y = p_contours->contours[i].points[j].y;
        }
        double contour_x_middle = ((max_x - min_x) / 2.0) + min_x;
        ROS_WARN_STREAM( "contour max_x: " << max_x << " min_x: " << min_x << " middle_x: " << contour_x_middle );

        //Iterate contour points and sum Y values found in top and bottom halves of the X range
        for ( size_t j = 0; j < points_size; j++ )
        {
            if ( p_contours->contours[i].points[j].x > contour_x_middle )
            {
                //top half of contour point
                avg_y_top += p_contours->contours[i].points[j].y;
                ++avg_top_count;
            }
            else
            {
                //bottom half of contour point
                avg_y_bottom += p_contours->contours[i].points[j].y;
                ++avg_bottom_count;
            }
        }

        if ( avg_bottom_count > 0 && avg_top_count > 0 )
        {
            avg_y_top /= avg_top_count;
            avg_y_bottom /= avg_bottom_count;
            ROS_WARN_STREAM( "avg Y top: " << avg_y_top << " avg Y bottom: " << avg_y_bottom );

            average_contour_difference += avg_y_top - avg_y_bottom;
        }
        else
        {
            ROS_INFO_STREAM( "avg bottom points: " << avg_bottom_count << " top points: " << avg_top_count << " total: " << points_size );
        }

    }
    average_contour_difference /= contours_count;

    ROS_ERROR_STREAM( "contours: " << contours_count << " avg contour difference: " << average_contour_difference );

    curve_detected = false;

    double threshold = 3.0;
    if ( average_contour_difference > threshold || average_contour_difference < -threshold )
    {
        curve_detected = true;

        double turn_angle_limit = 15.0;
        double turn_angle_ratio = 0.45;
        contour_turn_angle = average_contour_difference * turn_angle_ratio;

        if ( contour_turn_angle > turn_angle_limit ) contour_turn_angle = turn_angle_limit;
        if ( contour_turn_angle < -turn_angle_limit ) contour_turn_angle = -turn_angle_limit;
    }
    */

    /* Center contours in top half of image */
    double image_max_horizontal_pixels = 640.0; //TODO: not constant
    double image_max_verticle_pixels = 480.0; //TODO: not constant

    double average_top_horizonal_pos = 0.0;
    double average_bottom_horizonal_pos = 0.0;

    bool has_top_estimate = false;
    bool has_bottom_estimate = false;

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
            has_top_estimate = true;
            ++contours_in_top;
        }
        if ( avg_bottom_count > 0 )
        {
            avg_x_bottom /= avg_bottom_count;
            average_bottom_horizonal_pos += avg_x_bottom;
            has_bottom_estimate = true;
            ++contours_in_bottom;
        }

        ROS_WARN_STREAM( "avg X top: " << avg_x_top << " avg X bottom: " << avg_x_bottom << " num points: " << points_count );

    }
    average_top_horizonal_pos /= contours_in_top;
    average_top_horizonal_pos -= image_max_horizontal_pixels / 2.0;

    average_bottom_horizonal_pos /= contours_in_bottom;
    average_bottom_horizonal_pos -= image_max_horizontal_pixels / 2.0;

    ROS_ERROR_STREAM( "contours: " << contours_count << " avg top horiz pos: " << average_top_horizonal_pos << " avg botm horiz pos: " << average_bottom_horizonal_pos );

    //curve_detected = false;
    //double threshold = 0.0;
    //if ( average_top_horizonal_pos > threshold || average_top_horizonal_pos < -threshold )
    {
        curve_detected = true;

        double turn_angle_limit = 20.0;
        double turn_angle_ratio = -0.07;

        if ( has_top_estimate == true )
            contour_turn_angle = (0.9 * contour_turn_angle ) + ( 0.1 * (average_top_horizonal_pos * turn_angle_ratio));
        else if ( has_bottom_estimate == true ) //if we have no top detection but have bottom we might want to turn faster
            contour_turn_angle = (0.9 * contour_turn_angle ) + ( 0.1 * (average_bottom_horizonal_pos * turn_angle_ratio));

        if ( contour_turn_angle > turn_angle_limit ) contour_turn_angle = turn_angle_limit;
        if ( contour_turn_angle < -turn_angle_limit ) contour_turn_angle = -turn_angle_limit;

        ROS_WARN_STREAM( "turn angle: " << contour_turn_angle );
    }
}
void HROS5ContourFollower::publishVelocity(void)
{
    vel_pub_.publish(command_vel_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "contour_follower_node");
    HROS5ContourFollower contour_follower_node;

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
            if ( contour_follower_node.robot_has_fallen == true ) 
            {
                if ( stateFallen == false )
                {
                    stateFallen = true; //Flag to watch for fall recovered state
                    system( "espeak \"Shit. I've fallen over\"" );

                    contour_follower_node.current_state_ = STATE_FALLEN;

                    contour_follower_node.command_vel_.linear.x = 0.0;
                    contour_follower_node.command_vel_.angular.z = 0.0;
                    contour_follower_node.command_vel_.linear.y = 0.0;
                    contour_follower_node.publishVelocity();
                }
                continue;
            }
            else if ( stateFallen == true )
            {
                system( "espeak \"That's better!\"" );
                fallRecoveredStartTime = ros::Time::now();
                stateFallen = false;
            }

            switch ( contour_follower_node.current_state_ )
            {
                case STATE_INIT:
                    contour_follower_node.command_vel_.linear.x = 0.0;
                    contour_follower_node.command_vel_.angular.z = 0.0;
                    contour_follower_node.command_vel_.linear.y = 0.0;
                break;
                case STATE_FORWARD:
                    if ( contour_follower_node.command_vel_.linear.x < 25.0 ) contour_follower_node.command_vel_.linear.x += 0.5;
                    contour_follower_node.command_vel_.angular.z = 0.0;
                    contour_follower_node.command_vel_.linear.y = 0.0;
                    
                    if ( contour_follower_node.curve_detected == true )
                    {
                        contour_follower_node.command_vel_.angular.z = contour_follower_node.contour_turn_angle;
                    }
                    
                break;
                case STATE_FALLEN:
                    fallRecoveredNowTime = ros::Time::now();
                    fallRecoveredDuration = fallRecoveredNowTime - fallRecoveredStartTime;
                    if ( fallRecoveredDuration.sec > 5 )
                    {
                        contour_follower_node.current_state_ = STATE_INIT;
                        ROS_WARN( "State fallen is setting state init.. I've done as much as you've told me to do!" );
                        contour_follower_node.standRobotStartWalking();
                        contour_follower_node.current_state_ = STATE_FORWARD;
                    }
                break;
            }
            if ( ++publisherRate > 30 ) //Publish velocity at 1Hz. Walk gait will not change that quickly anyway.
            {
                contour_follower_node.publishVelocity();
                publisherRate = 0;
            }


        }
        last_time = current_time;
    }

    ros::waitForShutdown();
    return 0;
}
