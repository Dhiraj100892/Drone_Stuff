
// As compared to previous version..
// It is NOT sending random goal points... Its actually disabling the autopilot and sending the random velocity direction
// monitaring the error.. based on the /tum_ardrone/error msg
// monitoring the hit as well based on the accelerometer data
// In this robot will first rotate in place and will all ways try to hit the object in same orientation


// Status
// test - key_op..
// comment about the 0. velocity


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include <sstream>
#include "random_numbers/random_numbers.h"
#include "ardrone_autonomy/Navdata.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <sys/select.h>
#include "angles/angles.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

// for keyop pupose
int kfd = 0;
int num_hit = 0;
struct termios cooked, raw;

// for command
std::stringstream ss;
std_msgs::String msg;
ros::Publisher control_commands;
ros::Publisher velocity_commands;
ros::Publisher emergency_command;
ros::Duration dur(0.1);
bool read_hit = false;
bool autopilot_on = false;

// rand_num for theta generation
random_numbers::RandomNumberGenerator rand_num;
geometry_msgs::Twist vel_com;
// angles in degrees
float min_ang = 0;
float max_ang = 45;
// velocity with which it will travel the random point
float vel = 0.10;
double theta = 0.0;

// for state of the drone - random or towards center
bool towards_center = false;
bool key_op = false;
bool random_goal = false;
bool towards_new_theta = false;

int kbhit(void)
{
    struct timeval tv;
    fd_set read_fd;

    /* Do not wait at all, not even a microsecond */
    tv.tv_sec=0;
    tv.tv_usec=0;

    /* Must be done first to initialize read_fd */
    FD_ZERO(&read_fd);

    /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
    FD_SET(0,&read_fd);

    /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
    if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
        return 0;  /* An error occured */

    /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */

    if(FD_ISSET(0,&read_fd))
        /* Character pending on stdin */
        return 1;

    /* no characters were pending */
    return 0;
}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// Enable Autopilot
void enable_autopilot()
{
    key_op = false;
    random_goal = false;

    std::cout << "Enabling Auto Pilot" << std::endl;
    if(!autopilot_on)
    {
        ss.str("");
        ss << "c start";
        msg.data = ss.str();
        control_commands.publish(msg);
        ros::Duration(dur).sleep();
        autopilot_on = true;
    }
}

// Disable Autopilot
void disable_autopilot()
{
    std::cout << "Disabling Auto Pilot" << std::endl;
    if(autopilot_on)
    {
        ss.str("");
        ss << "c stop";
        msg.data = ss.str();
        control_commands.publish(msg);
        ros::Duration(dur).sleep();
        autopilot_on = false;
    }
}

// Initialize autopilot
void drone_init()
{
    //
    std::cout << "stdring autopilot" << std::endl;
    autopilot_on = true;
    ss.str("");
    ss << "c start";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    std::cout << "starting to fly \n" << std::endl;

    ss.str("");
    ss << "c autoInit 500 800 4000 0.5";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    ss.str("");
    ss << "c setReference $POSE$";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    ss.str("");
    ss << "c setInitialReachDist 0.2";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    ss.str("");
    ss << "c setStayWithinDist 0.1";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    ss.str("");
    ss << "c setStayTime 6";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    ss.str("");
    ss << "c lockScaleFP";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    // send center command
    ss.str("");
    ss << "c goto 0.0 0.0 0.5 0.0";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

}

// Function to be called for emergency condition
void drone_emergency()
{
    std::cout << "Emegency Landing" << std::endl;
    std_msgs::Empty msg;
    emergency_command.publish(msg);
}

// Function to be called for landing drone
void drone_land()
{
    enable_autopilot();

    std::cout << "Landing \n" << std::endl;

    // clear all the commands
    ss.str("");
    ss << "c clearCommands";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    // send land command
    ss.str("");
    ss << "c land";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

}

// set velocity
void set_vel_command(float x, float y, float z, float yaw)
{
    disable_autopilot();
    towards_center = false;

    vel_com.linear.x = x ;
    vel_com.linear.y = y ;
    vel_com.linear.z = z ;
    vel_com.angular.x = 0.0;
    vel_com.angular.y = 0.0;
    vel_com.angular.z = yaw;
}

// function to be called for sending go to center command
void go_to_center()
{
    enable_autopilot();

    towards_center = true;
    towards_new_theta = false;
    random_goal = false;

    std::cout << "Going to origin" << std::endl;
    ss.str("");
    ss << "c clearCommands";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();

    // send center command
    ss.str("");
    ss << "c goto 0.0 0.0 0.5 " << theta;
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();
}

// funtion to be called for moving to random point
void go_to_rand_point()
{
    // first it will orient with new theta.. and then will try to move in th eback direction
    if(!towards_new_theta)
    {
        std::cout << "Next command \n" << std::endl;
        theta = rand_num.uniformReal(min_ang, max_ang);
        std::cout << "Theta = " << theta << std::endl;
        std::cout << "Auto Controller.. Yaw Mode" << std::endl;

        ss.str("");
        ss << "c clearCommands";
        msg.data = ss.str();
        control_commands.publish(msg);
        ros::Duration(dur).sleep();

        // send center command
        ss.str("");
        ss << "c goto 0.0 0.0 0.5 " << theta;
        msg.data = ss.str();
        control_commands.publish(msg);
        ros::Duration(dur).sleep();

        towards_new_theta = true;
        towards_center = true;
    }
    else
    {
        std::cout << "No Auto Controller" << std::endl;
        random_goal = true;
        key_op = false;
        set_vel_command(-vel,0.0,0.0,0.0);
        towards_new_theta = false;
        towards_center = false;
    }
}



// navdata callback
void nav_callback(const ardrone_autonomy::NavdataConstPtr &msg)
{
    float thr_peak = 0.3;
    bool hit = false;
    float value = msg->ax * msg->ax + msg->ay * msg->ay;

    if ( value > thr_peak )
    {
        hit = true;
        num_hit++;
        if ( read_hit ) go_to_center();
        std::cout <<"ax = " << std::setprecision(1) << msg->ax <<
                    " ay = " << std::setprecision(1) << msg->ay <<
                    " value = " << std::setprecision(1) << value <<
                    " Hit = " << hit <<
                    " Num Hit = " << num_hit << std::endl;
    }


}

//error callback
void err_callback(const std_msgs::Float32ConstPtr &msg)
{
//    float pose_error_thr = 0.2;
//    float pose_error = msg->data;
//    std::cout << "Pose_Error = " << pose_error << std::endl;

//    // take action according to the pre_command
//    if(pose_error < pose_error_thr && read_hit)
//    {
//        if(towards_center)
//        {
//            std::cout << "Going toward random point" << std::endl;
//            go_to_rand_point();
//        }
//        //        else
//        //        {
//        //            std::cout << "Going toward Center" << std::endl;
//        //            go_to_center();
//        //        }

//    }
}

// error pose callback
void err_pose_callback(const geometry_msgs::TwistConstPtr msg)
{
    // need to do stuffs depending on the drone behaviour
    float pose_error_thr = 0.2;
    float yaw_err_mul_fact = 0.05;
    float pose_error = std::pow ( std::pow(msg->linear.x,2) + std::pow(msg->linear.y,2) + std::pow(msg->linear.z,2) +
            yaw_err_mul_fact * std::pow(msg->angular.z,2) , 0.5 ) ;

    std::cout<< "Error in Yaw = " << msg->angular.z <<"error = " << pose_error <<std::endl;
    // take action according to the pre_command
    if(pose_error < pose_error_thr && read_hit)
    {
        if(towards_center)
        {
            std::cout << "Going toward random point" << std::endl;
            go_to_rand_point();
        }

    }
}

int main(int argc, char **argv)
{

    //for key op
    signal(SIGINT,quit);

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use keys to move the bebop.");



    ros::init(argc, argv, "control_node");

    ros::NodeHandle n;

    control_commands = n.advertise<std_msgs::String>("/tum_ardrone/com", 1);
    velocity_commands = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    emergency_command = n.advertise<std_msgs::Empty>("/ardrone/reset",1);
    ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata",1,nav_callback);
    ros::Subscriber err_sub = n.subscribe("/tum_ardrone/error",1,err_callback);
    ros::Subscriber pose_err_sub = n.subscribe("/tum_ardrone/error_pose",1,err_pose_callback);

    //client
    ros::ServiceClient flat_trim_client = n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

    //flat_trim_client.call();
    ros::Duration(dur).sleep();

    // for the rate of while loop
    ros::Rate r(100); // 10 hz

    while (ros::ok())
    {
        ros::spinOnce();

//        // intialize the velocity if key op is on and random goal is off
//        if(key_op && !random_goal)
//        {
//            set_vel_command(0.0,0.0,0.0,0.0);
//        }

        if (kbhit())
        {
            char c = std::getchar();

            // if e is preseed.. make the emergency land
            if (c == 'e')
            {
                read_hit = false;
                drone_emergency();
            }
            // if d is pressed.. take oer the control and land
            else if (c == 'd')
            {
                read_hit = false;
                drone_land();
            }

            //if 's' is pressed.. starts and send command
            else if(c == 's')
            {
                read_hit = false;
                drone_init();
            }

            // if 'o' go to the origin
            else if(c == 'o')
            {
                go_to_center();
            }

            // if 'n' next target
            else if( c == 'n')
            {
                read_hit = true;
                go_to_rand_point();
            }

            else if( c == 'i')
            {
                std::cout << "moving in forward direction" << std::endl;
                read_hit = true;
                set_vel_command(vel,0.0,0.0,0.0);
                key_op = true;
                random_goal = false;
            }

            else if( c == 'k')
            {
                std::cout << "moving in backward direction" << std::endl;
                read_hit = true;
                set_vel_command(-vel,0.0,0.0,0.0);
                key_op = true;
                random_goal = false;
            }
            else if( c == 'j')
            {
                std::cout << "moving in left direction" << std::endl;
                read_hit = true;
                set_vel_command(0.0,-vel,0.0,0.0);
                key_op = true;
                random_goal = false;
            }
            else if( c == 'l')
            {
                std::cout << "moving in right direction" << std::endl;
                read_hit = true;
                set_vel_command(0.0,vel,0.0,0.0);
                key_op = true;
                random_goal = false;
            }
            else if(c == 't')
            {
                std::cout << "Hovering" << std::endl;
                read_hit = true;
                set_vel_command(0.0,0.0,0.0,0.0);
                key_op = true;
                random_goal = false;
            }



        }

        if(!autopilot_on && !towards_center && read_hit&& !towards_new_theta)
        {
//            std::cout << "Velocity Commands are sent" << std::endl;
            velocity_commands.publish(vel_com);
            r.sleep();
        }

    }


    return 0;
}
