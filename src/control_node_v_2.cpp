// As compared to previous version..
// It is sending random goal points..
// monitaring the error.. based on the /tum_ardrone/error msg
// monitoring the hit as well based on the accelerometer data

// Status
// Not tested any thinng

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

// for keyop pupose
int kfd = 0;
int num_hit = 0;
struct termios cooked, raw;

// for command
std::stringstream ss;
std_msgs::String msg;
ros::Publisher control_commands;
ros::Duration dur(0.1);
bool read_hit = false;

// rand_num for theta generation
random_numbers::RandomNumberGenerator rand_num;

// radius
double rad = 2.2;

// for state of the drone - random or towards center
bool towards_center = false;

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

void drone_init()
{
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

void drone_land()
{
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

void go_to_center()
{
    towards_center = true;

    std::cout << "Going to origin" << std::endl;
    ss.str("");
    ss << "c clearCommands";
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

void go_to_rand_point()
{
    towards_center = false;

    std::cout << "Next command \n" << std::endl;
    double theta;
    theta = rand_num.uniformReal(0, 2 * M_PI);
    std::cout << "Theta = " << theta * 180 / M_PI << std::endl;
    double x = rad * std::cos(theta);
    double y = rad * std::sin(theta);
    double z = 0.5;
    double yaw = 0.0;

    ss.str("");
    ss << "c clearCommands";
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();
    ss.str("");

    ss.str("");
    ss << "c goto " << x << " " << y << " " << (double)z << " " << (double)yaw;
    msg.data = ss.str();
    control_commands.publish(msg);
    ros::Duration(dur).sleep();
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
    float pose_error_thr = 0.2;
    float pose_error = msg->data;
    std::cout << "Pose_Error = " << pose_error << std::endl;

    // take action according to the pre_command
    if(pose_error < pose_error_thr && read_hit)
    {
        if(towards_center)
        {
            std::cout << "Going toward random point" << std::endl;
            go_to_rand_point();
        }
        else
        {
            std::cout << "Going toward Center" << std::endl;
            go_to_center();
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
    ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata",1,nav_callback);
    ros::Subscriber err_sub = n.subscribe("//tum_ardrone/error",1,err_callback);

    //client
    ros::ServiceClient flat_trim_client = n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

    //flat_trim_client.call();
    ros::Duration(dur).sleep();


    while (ros::ok())
    {
        ros::spinOnce();

        if (kbhit())
        {
            char c = std::getchar();

            // if d is pressed.. take oer the control and land
            if (c == 'd')
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

            // if 't' move by
            else if(c == 't')
            {
                read_hit = true;
                std::cout << "Test Key" << std::endl;
                ss.str("");
                ss << "c clearCommands";
                msg.data = ss.str();
                control_commands.publish(msg);
                ros::Duration(dur).sleep();
                ss.str("");

                ss << "c goto 0.0 -1.0 0.5 0.0";
                msg.data = ss.str();
                control_commands.publish(msg);
                ros::Duration(dur).sleep();
            }


            // if 'm' move by
            else if(c == 'm')
            {
                read_hit = true;
                std::cout << "Move Key" << std::endl;
                ss.str("");
                ss << "c moveByRel 0.0 -0.1 0.0 0.0";
                msg.data = ss.str();
                control_commands.publish(msg);
                ros::Duration(dur).sleep();
            }


            // if 'n' next target
            else if( c == 'n')
            {
                read_hit = true;
                go_to_rand_point();
            }
        }

    }


    return 0;
}
