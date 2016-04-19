// this code is just for sending random goal commands to the drone.
// Random goals will lies on circle of radius R

// Status
// Not tested any thinng

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include <sstream>
#include "random_numbers/random_numbers.h"
#include "ardrone_autonomy/Navdata.h"

void nav_callback(const ardrone_autonomy::NavdataConstPtr &msg)
{
    std::cout << "ax = " << msg->ax << std::endl;
    std::cout << "ay = " << msg->ay << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control_node");

    ros::NodeHandle n;

    ros::Publisher control_commands = n.advertise<std_msgs::String>("/tum_ardrone/com", 1);
    ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata",1,nav_callback);

    std_msgs::String msg;

    std::stringstream ss;
    ros::Duration dur(0.1);

    //client
    ros::ServiceClient flat_trim_client = n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");

    //flat_trim_client.call();
    ros::Duration(dur).sleep();

    // rand_num for theta generation
    random_numbers::RandomNumberGenerator rand_num;

    // radius
    double rad = 2.0;



    while (ros::ok())
    {
        char c = std::getchar();
        ros::spinOnce();

        // if d is pressed.. take oer the control and land
        if (c == 'd')
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

        //if 's' is pressed.. starts and send command
        else if(c == 's')
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
            ss << "c setStayTime 5";
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

        // if 'o' go to the origin
        else if(c == 'o')
        {
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

        // if 't' move by
        else if(c == 't')
        {

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
            std::cout << "Test Key" << std::endl;
        }


        // if 'm' move by
        else if(c == 'm')
        {
            ss.str("");
            ss << "c moveByRel 0.0 -0.1 0.0 0.0";
            msg.data = ss.str();
            control_commands.publish(msg);
            ros::Duration(dur).sleep();
        }


        // if 'n' next target
        else if( c == 'n')
        {
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

    }


    return 0;
}
