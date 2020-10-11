// Kill Turtle
// This little game let you hunt a turtle in turtlesim.
// If you get turtle1 close enough to turtle2, it will
// kill turtle2.
//
// Copyright 2019, Karl D. Hansen

#include <cstdlib>
#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>

ros::NodeHandle * p_node_handle;
turtlesim::Pose turtle2_pose;
const float kill_distance = 0.1;

void kill_turtle(int n)
{
    std::stringstream ss;
    ss << "turtle" << n;
    turtlesim::Kill kill_message;
    kill_message.request.name = ss.str();
    ros::ServiceClient kill_client
        = p_node_handle->serviceClient<turtlesim::Kill>("/kill");
    kill_client.call(kill_message);
}

void spawn_turtle(int n)
{
    ros::ServiceClient spawn_client = 
        p_node_handle->serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_message;
    spawn_message.request.name = "turtle2";
    spawn_message.request.x = rand() % 11;
    spawn_message.request.y = rand() % 11;
    spawn_client.call(spawn_message);
}

void cbPose1(const turtlesim::Pose::ConstPtr& message)
{
    double x_distance = turtle2_pose.x - message->x;
    double y_distance = turtle2_pose.y - message->y;
    double distance = hypot(x_distance, y_distance);
    // If the distance is close enough, kill the turtle
    // then respawn a new one.
    if (distance < kill_distance)
    {
        kill_turtle(2);
        // Set the saved pose outside the area until
        // we get a new pose.
        turtle2_pose.x = -10.0;
        spawn_turtle(2);
    }
}

void cbPose2(const turtlesim::Pose::ConstPtr& message)
{
    turtle2_pose = *message;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kill_turtles");

    // Create a node handle and save a reference in the node handle
    // pointer for use outside main.
    ros::NodeHandle node_handle;
    p_node_handle = &node_handle;

    // Wait a while for the turtlesim to load and then spawn
    // a new turtle.
    ros::Duration(1.0).sleep();
    spawn_turtle(2);

    // Subscribe to the poses of both turtle1 and turtle2 in order
    // to compare them.
    ros::Subscriber pose1_subscriber
        = node_handle.subscribe("/turtle1/pose", 1, &cbPose1);
    ros::Subscriber pose2_subscriber
        = node_handle.subscribe("/turtle2/pose", 1, &cbPose2);

    ros::spin();

    return 0;
}
