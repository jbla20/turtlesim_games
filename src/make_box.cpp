#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <stdlib.h>
#include <time.h>


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "move_turtlesim");

  srand (time(NULL));
  
  int randomNumber = rand() % 10 + 1;

  ROS_INFO("Random Number: %d", randomNumber);
  
  float box_size = ros::param::param("~box_size", randomNumber);

  ros::NodeHandle nh;
    ros::Duration(2.0).sleep();
  ros::service::waitForService("/turtle1/teleport_absolute", -1);

  ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
  ros::ServiceClient pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");


  turtlesim::SetPen pen_srv;
  pen_srv.request.off = true;
  pen_client.call(pen_srv);

  std::cout << "h1";

  turtlesim::TeleportAbsolute srv;

  srv.request.x = 5.5-box_size/2;
  srv.request.y = 5.5-box_size/2;
  teleport_client.call(srv);

  std::cout << "h2";

  int randomNumberR = rand() % 255 + 1;
  int randomNumberG = rand() % 255 + 1;
  int randomNumberWidth = rand() % 20 + 1;

  ROS_INFO("Random Number Red: %d", randomNumberR);
  ROS_INFO("Random Number Green: %d", randomNumberG);
  ROS_INFO("Random Number Width: %d", randomNumberWidth);

  pen_srv.request.off = false;
  pen_srv.request.width = randomNumberWidth;
  pen_srv.request.r = randomNumberR;
  pen_srv.request.g = randomNumberG;
  pen_client.call(pen_srv);

  srv.request.x = 5.5-box_size/2;
  srv.request.y = 5.5+box_size/2;
  teleport_client.call(srv);

  srv.request.x = 5.5+box_size/2;
  srv.request.y = 5.5+box_size/2;
  teleport_client.call(srv);

  srv.request.x = 5.5+box_size/2;
  srv.request.y = 5.5-box_size/2;
  teleport_client.call(srv);

  srv.request.x = 5.5-box_size/2;
  srv.request.y = 5.5-box_size/2;
  teleport_client.call(srv);

  pen_srv.request.off = true;
  pen_client.call(pen_srv);

  srv.request.x = 5.5;
  srv.request.y = 5.5;
  teleport_client.call(srv);

  pen_srv.request.off = false;
  pen_srv.request.width = 4;
  pen_srv.request.r = 10;
  pen_srv.request.g = 130;
  pen_srv.request.b = 200;
  pen_client.call(pen_srv);

  return 0;
}