#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Color.h>
#include <turtlesim/SetPen.h>

ros::Publisher cmd_vel_pub;

bool first_color_saved = false;
turtlesim::Color first_color;

// Callback function to handle incomming messages on the color_sensor topic.
// First the default color is saved, then the following messages are compared to
// that color. If the color is different, the robot turns.
void cbColor(turtlesim::Color sensed_color)
{
  if(!first_color_saved)
  {
    first_color = sensed_color;
    first_color_saved = true;
  }

  if(   sensed_color.r != first_color.r
     && sensed_color.g != first_color.g
     && sensed_color.b != first_color.b)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 2.0;
    cmd_vel_pub.publish(msg);
    ros::Duration(1.0).sleep();
  }
  return;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "move_circle");

  // Get the node handle to interact with the ROS network
  ros::NodeHandle node_handle;
    ros::Duration(4.0).sleep();
  // Turn off the pen. Otherwise, the turtle will sense the color it just
  // painted.
  turtlesim::SetPen pen_srv;
  pen_srv.request.off = true;
  ros::ServiceClient pen_client =
    node_handle.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  pen_client.call(pen_srv);

  // Subscribe to the color sensor of the turtle
  ros::Subscriber color_sub =
    node_handle.subscribe("/turtle1/color_sensor", 1, cbColor);

  // Advertise to the ROS network that this node wil have velocity commands for
  // the turtle
  cmd_vel_pub =
    node_handle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // Initially the turtle is commanded to move straight.
  geometry_msgs::Twist msg;
  msg.linear.x = 1.0;
  msg.angular.z = 0.0;

  // This is where it happens. We continously send the command to the turtle and
  // sleep for 200 ms. So the commands are send to the turtle at 5 Hz. This is
  // fast enough for the "emergency brake" not to activate.
  // We allso spin the ROS event loop. This is to read any incomming messages on
  // the subscriptions. This is where the color_callback will be called if there
  // is a message for it.
  while(ros::ok())
  {
    cmd_vel_pub.publish(msg);
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  return 0;
}