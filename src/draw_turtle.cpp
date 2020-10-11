#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <iostream>
#include "stdlib.h"
#include "time.h"
#include <cmath>

#define MAXDIMENSIONS 2

void initTurtlePosition(float x, float y, ros::ServiceClient teleport_client, ros::ServiceClient pen_client, turtlesim::TeleportAbsolute srv, turtlesim::SetPen pen_srv)
{
    pen_srv.request.off = true;
    pen_client.call(pen_srv);
    srv.request.x = x;
    srv.request.y = y;
    teleport_client.call(srv);
    pen_srv.request.off = false;
    pen_srv.request.width = 2;
    pen_client.call(pen_srv);
}

void moveTurtle(float x, float y, ros::ServiceClient teleport_client, turtlesim::TeleportAbsolute srv)
{
    srv.request.x = x;
    srv.request.y = y;
    teleport_client.call(srv);
}

int fillCoordinates(int sizeCoordA, float coordinates[][2])
{
    if (sizeCoordA < 4)
        return 0;

    srand(time(NULL));
    // random number between 4 and the size of the array (sizeCoordA)
    int numOfCoordinates = rand() % (sizeCoordA - 3) + 4;

    for (int i = 0; i < numOfCoordinates; i++)
    {
        //int randomNumberX = rand() % 10 + 1;
        //int randomNumberY = rand() % 10 + 1;
        float randomNumberX = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * 11.5;
        float randomNumberY = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * 11.5;
        coordinates[i][0] = randomNumberX;
        coordinates[i][1] = randomNumberY;
    }

    coordinates[numOfCoordinates - 1][0] = coordinates[0][0];
    coordinates[numOfCoordinates - 1][1] = coordinates[0][1];

    return numOfCoordinates;
}

/*float getManhattanDistance(float point[2], float targetPoint[2])
{
    return abs(targetPoint[0] - point[0]) + abs(targetPoint[1] - point[1]);
}*/

void swapCoordinatesByManhattenDistance(int numOfCoordinates, float array[][2])
{
    float distanceBetweenPointsByManhatten[numOfCoordinates];

    for (int i = 0; i < numOfCoordinates; i++)
    {
        distanceBetweenPointsByManhatten[i] = (abs(0.0 - array[i][0])) + abs(0.0 - array[i][1]);
    }
    bool swapped{true};

    while (swapped)
    {
        swapped = false;
        for (int i = 0; i < numOfCoordinates - 1; i++)
        {
            if (distanceBetweenPointsByManhatten[i] > distanceBetweenPointsByManhatten[i + 1])
            {
                std::swap(array[i], array[i + 1]);
                float temp = distanceBetweenPointsByManhatten[i];
                distanceBetweenPointsByManhatten[i] = distanceBetweenPointsByManhatten[i + 1];
                distanceBetweenPointsByManhatten[i + 1] = temp;
                swapped = true;
            }
        }
    }
}

void printArray(int num, float array[][2])
{
    for (int i = 0; i < num; i++)
    {
        std::cout << std::fixed << std::setprecision(3)
                  << "x: " << std::right << std::setw(6) << array[i][0] << std::setw(6)
                  << "y: " << std::right << std::setw(6) << array[i][1] << "\n";
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "draw_turtle");
    ros::NodeHandle nh;
    ros::service::waitForService("/turtle1/teleport_absolute", -1);
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute srv;
    ros::ServiceClient pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen pen_srv;
    ros::Rate loop_rate(2);

    int numOfEle = 10;
    float coordinates[numOfEle][2];

    for (int i = 0; i < numOfEle; i++)
    {
        coordinates[i][0] = 0;
        coordinates[i][1] = 0;
    }

    std::cout << "Randomly generated coordinates:\n";

    int numOfCoordinates = fillCoordinates(numOfEle, coordinates);
    printArray(numOfCoordinates, coordinates);

    std::cout << "\n\n"
              << "Coordinates sorted by the Manhatten Distance between point and Origin(0,0):\n";

    swapCoordinatesByManhattenDistance(numOfCoordinates, coordinates);
    printArray(numOfCoordinates, coordinates);

    float startX = 5.544445, startY = 5.544445;
    if (argc > 1)
    {
        startX = atof(argv[1]);
        startY = atof(argv[2]);
    }

    initTurtlePosition(coordinates[0][0], coordinates[0][1], teleport_client, pen_client, srv, pen_srv);
    loop_rate.sleep();

    for (int i = 0; i < numOfCoordinates; i++)
    {
        moveTurtle(coordinates[i][0], coordinates[i][1], teleport_client, srv);
        loop_rate.sleep();
    }

    /* Exercise 2
  pen_srv.request.off = true;
  pen_client.call(pen_srv);
  srv.request.x = startX;
  srv.request.y = startY;
  teleport_client.call(srv);
  pen_srv.request.off = false;
  pen_srv.request.width = 2;
  pen_client.call(pen_srv);
  loop_rate.sleep();

  int sizeCoordA = 5;
  int sizeCoordB = 2;
  float coordinates[sizeCoordA][sizeCoordB] = {{startX+5, startY},{startX+5, startY+4},{startX+2, startY+4},{startX,startY+6},{startX, startY}};

  for(int i=0; i<sizeCoordA; i++){
    srv.request.x = coordinates[i][0];
    srv.request.y = coordinates[i][1];
    teleport_client.call(srv);
    loop_rate.sleep();
  }

  */

    // Exercise 1
    /*int sizeCoordA = 5;
    int sizeCoordB = 2;

    for (int i = 0; i < sizeCoordA; i++)
    {
        srv.request.x = fillCoordinates(sizeCoordA, coordinates);
        srv.request.y = fillCoordinates(sizeCoordA, coordinates);
        teleport_client.call(srv);
        loop_rate.sleep();
    }
*/
    return 0;
}