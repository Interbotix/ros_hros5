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

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_S 0x73
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_X 0x78


#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e


#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43


#define KEYCODE_ARROW_1 0x1b
#define KEYCODE_ARROW_2 0x5b

#define COMMAND_TIMEOUT_SEC 0.2

// at full joystick depression you'll go this fast
double max_speed = 0.500; // m/second
double max_turn = 60.0*M_PI/180.0; // rad/second
// should we continuously send commands?
bool always_command = false;


class TBK_Node
{
private:
    geometry_msgs::Twist cmdvel;
    std_msgs::Float64 angle_msg;
    std_msgs::Int32 action_index_msg;
    std_msgs::Int32 enable_walking_msg;
    std_msgs::Bool sit_stand_msg;

    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher tilt_pub_;
    ros::Publisher pan_pub_;
    ros::Publisher action_pub_;
    ros::Publisher enable_walking_pub_;
    ros::Publisher sit_stand_pub_;

    ros::Subscriber joint_states_sub_;

    bool walking_enabled;
    float pan, tilt;

public:
    TBK_Node()
    {
        walking_enabled = false;
        pan = 0.0;
        tilt = 0.0;

        joint_states_sub_ = n_.subscribe("/hros5/joint_states", 100, &TBK_Node::jointStatesCb, this);


        pub_ = n_.advertise<geometry_msgs::Twist>("/hros5/cmd_vel",1);
        tilt_pub_ = n_.advertise<std_msgs::Float64>("/hros5/HeadPitch_position_controller/command", 100);
        pan_pub_ = n_.advertise<std_msgs::Float64>("/hros5/HeadYaw_position_controller/command", 100);
        action_pub_ = n_.advertise<std_msgs::Int32>("/hros5/start_action", 100);
        enable_walking_pub_ = n_.advertise<std_msgs::Int32>("/hros5/enable_walking", 100);
        sit_stand_pub_ = n_.advertise<std_msgs::Bool>("/hros5/standing_sitting", 100);
    }
    ~TBK_Node() { }
    void keyboardLoop();
    void stopRobot()
    {
        cmdvel.linear.x = cmdvel.angular.z = 0.0;
        pub_.publish(cmdvel);
    }


    void jointStatesCb(const sensor_msgs::JointState& msg)
    {
        pan = msg.position.at(1); //TODO: msg.position.at unsafe to configuration changes
        tilt = msg.position.at(0); //TODO: msg.position.at unsafe to configuration changes
        //ROS_INFO("Teleop Pan: %0.4f Tilt: %0.4f", pan, tilt);
    }
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{



    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    TBK_Node tbk;

    boost::thread t = boost::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void
TBK_Node::keyboardLoop()
{
    char keyboard_input;
    double max_tv = max_speed;
    double max_rv = max_turn;
    bool dirty=false;

    int speed=0;
    int turn=0;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("q/z : increase/decrease max angular and linear speeds by 10%");
    puts("w/x : increase/decrease max linear speed by 10%");
    puts("e/c : increase/decrease max angular speed by 10%");
    puts("---------------------------");
    puts("Moving around:");
    puts("   u    i    o");
    puts("   j    k    l");
    puts("   m    ,    .");
    puts("1/2 kick left/right");
    puts("3/4 get up from forward/backwards falll");
    puts("arrow keys move head");
    puts("---------------------------");
    puts("6 stand up action");
    puts("7 sit down action");
    puts("9 start walking gait");
    puts("0 stop walking gait" );
    puts("---------------------------");
    puts("anything else : stop");
    puts("---------------------------");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;
        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &keyboard_input, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
            continue;

        switch ( keyboard_input ) {
        case KEYCODE_I:
            ROS_INFO("i Go straight without rotation");
            speed = 1;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_J:
            ROS_INFO("j Turn left at a fixed position");
            speed = 0;
            turn = 1;
            dirty = true;
            break;
        case KEYCODE_COMMA:
            ROS_INFO(", Go backward without rotation");
            speed = -1;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_O:
            ROS_INFO("O Move forward with right turn");
            speed = 1;
            turn = -1;
            dirty = true;
            break;
        case KEYCODE_L:
            ROS_INFO("l Turn right at a fixed position");
            speed = 0;
            turn = -1;
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_INFO("U Move forward with left turn");
            speed = 1;
            turn = 1;
            dirty = true;
            break;
        case KEYCODE_M:
            ROS_INFO("m Move backward with clockwise rotation");
            speed = -1;
            turn = -1;
            dirty = true;
            break;
        case KEYCODE_PERIOD:
            ROS_INFO(". Move backward with counter clockwise rotation");
            speed = 1;
            turn = -1;
            dirty = true;
            break;
        case KEYCODE_K:
            ROS_INFO(" k Stop");
            speed = 0;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_Q:
            ROS_INFO("q Increase angular and linear speeds by 10 percent");
            max_tv += max_tv / 10;
            max_rv += max_rv / 10;
            if(always_command)
                dirty=true;
            break;
        case KEYCODE_Z:
            ROS_INFO("Z Decrease angular and linear speeds 10 percent");
            max_tv -= max_tv / 10;
            max_rv -= max_rv / 10;
            if(always_command)
                dirty=true;
            break;
        case KEYCODE_W:
            ROS_INFO("w Increase linear speed by 10 percent");
            max_tv += max_tv / 10;
            if(always_command)
                dirty=true;
            break;
        case KEYCODE_X:
            ROS_INFO("X Decrease linear speed by 10 percent");
            max_tv -= max_tv / 10;
            if(always_command)
                dirty=true;
            break;
        case KEYCODE_E:
            ROS_INFO("e Increase angular speed by 10 percent");
            max_rv += max_rv / 10;
            if(always_command)
                dirty=true;
            break;
        case KEYCODE_C:
            ROS_INFO("C Decrease angular speed by 10 percent");
            max_rv -= max_rv / 10;
            if(always_command)
                dirty=true;
            break;
        case KEYCODE_9:
            ROS_INFO("9 enable walking");
            enable_walking_msg.data = 1;
            enable_walking_pub_.publish(enable_walking_msg);
            break;
        case KEYCODE_0:
            ROS_INFO("0 disable walking");
            enable_walking_msg.data = 0;
            enable_walking_pub_.publish(enable_walking_msg);
            break;
        case KEYCODE_1:
            ROS_INFO("1 kick left");
            action_index_msg.data = 13;
            action_pub_.publish(action_index_msg);
            break;
        case KEYCODE_2:
            ROS_INFO("2 kick right");
            action_index_msg.data = 12;
            action_pub_.publish(action_index_msg);
            break;
        case KEYCODE_3:
            ROS_INFO("3 get up from forward fall");
            action_index_msg.data = 10;
            action_pub_.publish(action_index_msg);
            break;
        case KEYCODE_4:
            ROS_INFO("4 get up from backwards fall");
            action_index_msg.data = 11;
            action_pub_.publish(action_index_msg);
            break;
        case KEYCODE_6:
            ROS_INFO("6 stand up action");
            sit_stand_msg.data = 1;
            sit_stand_pub_.publish(sit_stand_msg);
            break;
        case KEYCODE_7:
            ROS_INFO("7 sit down action");
            sit_stand_msg.data = 0;
            sit_stand_pub_.publish(sit_stand_msg);
            break;
        case KEYCODE_UP:
            ROS_INFO("head up");
            if ( tilt < 0.24 ) angle_msg.data = tilt+0.05; //User determined limit
            else angle_msg.data = tilt;
            tilt_pub_.publish(angle_msg);
            break;
        case KEYCODE_DOWN:
            ROS_INFO("head down");
            if ( tilt > -0.35 ) angle_msg.data = tilt-0.05; //User determined limit
            else angle_msg.data = tilt;
            tilt_pub_.publish(angle_msg);
            break;
        case KEYCODE_RIGHT:
            ROS_INFO("head right");
            angle_msg.data = pan-0.05;
            pan_pub_.publish(angle_msg);
            break;
        case KEYCODE_LEFT:
            ROS_INFO("head left");
            angle_msg.data = pan+0.05;
            pan_pub_.publish(angle_msg);
            break;
        case KEYCODE_ARROW_1:
            break;
        case KEYCODE_ARROW_2:
            break;
        default:
            ROS_INFO("default");
            speed = 0;
            turn = 0;
            dirty = true;
        }

       // std::cout << std::hex;
       // std::cout << (int)keyboard_input   << std::endl;
        if (dirty == true)
        {
            cmdvel.linear.x = speed * max_tv;
            cmdvel.angular.z = turn * max_rv;
            pub_.publish(cmdvel);
            dirty = false;
        }
    }
}
