#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <iostream>
#include "stdlib.h"
#include "time.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_turtle");

    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    ros::Rate loop_rate(10);

    ros::ServiceClient pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    turtlesim::TeleportAbsolute srv;

    turtlesim::SetPen pen_srv;
    pen_srv.request.off = true;
    pen_client.call(pen_srv);

    pen_srv.request.off = false;
    pen_srv.request.width = 2;
    pen_srv.request.r = 255;
    pen_srv.request.g = 255;
    pen_srv.request.b = 255;
    pen_client.call(pen_srv);

    geometry_msgs::Twist twist;

    int userInput = 1;
    int numberShape = 1;

    std::cout << "Type 1 for: circle \n";
    std::cout << "Type 2 for: square \n";
    std::cout << "Type 3 for: triangle \n";
    std::cout << "Enter number: ";
    std::cin >> numberShape;

    while (ros::ok() && userInput != 0)
    {

        switch (numberShape)
        {
        case 1:
            for (int i = 0; i < 2 * 10; i++)
            {
                twist.linear.x = 2.0 * M_PI;
                twist.angular.z = M_PI;
                vel_pub.publish(twist);
                loop_rate.sleep();
            }
            for (int j = 0; j < 10; j++)
            {
                loop_rate.sleep();
            }
            break;

        case 2:
            for (int t = 0; t < 4; t++)
            {
                for (int i = 0; i < 15; i++)
                {
                    twist.linear.x = 2.0;
                    twist.angular.z = 0.0;
                    vel_pub.publish(twist);
                    loop_rate.sleep();
                }
                for (int i = 0; i < 10; i++)
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = M_PI_2;
                    vel_pub.publish(twist);
                    loop_rate.sleep();
                }
            }
            for (int j = 0; j < 10; j++)
            {
                loop_rate.sleep();
            }
            break;

        case 3:
            for (int t = 0; t < 3; t++)
            {
                for (int i = 0; i < 15; i++)
                {
                    twist.linear.x = 2.0;
                    twist.angular.z = 0.0;
                    vel_pub.publish(twist);
                    loop_rate.sleep();
                }
                for (int i = 0; i < 10; i++)
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = M_PI * (2.0 / 3.0);
                    vel_pub.publish(twist);
                    loop_rate.sleep();
                }
            }
            for (int j = 0; j < 10; j++)
            {
                loop_rate.sleep();
            }
            break;

            {
            default:
                std::cout << "You have entered " << numberShape << " which was not one of the options. Try again.\n";
            }
        }

        pen_srv.request.off = true;
        pen_client.call(pen_srv);

        srv.request.x = 5.5;
        srv.request.y = 5.5;
        teleport_client.call(srv);

        std::cout << "\nContinue (yes=1/no=0)? ";
        std::cin >> userInput;

        if (userInput == 0)
        {
            break;
        }

        std::cout << "\nEnter new number: ";
        std::cin >> numberShape;
    }

    return 0;
}
