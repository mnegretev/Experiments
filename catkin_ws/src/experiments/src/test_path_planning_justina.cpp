#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "justina_tools/JustinaNavigation.h"

#define SM_GET_RANDOM_GOALS 0
#define SM_SEND_TO_GOAL     10
#define SM_WAIT_FOR_GOAL_REACHED 20
#define SM_GOAL_REACHED   30

bool collision_detected = false;
float collision_x = 0;
float collision_y = 0;
bool global_goal_reached = false;

void callback_goal_reached(const std_msgs::Bool::ConstPtr& msg)
{
    global_goal_reached = true;
}

void callback_collision(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    collision_detected = true;
    collision_x = msg->point.x;
    collision_y = msg->point.y;
}


int main(int argc, char** argv)
{	
    std::cout << "INITIALIZING TEST OF PATH PLANNING ALGORITHMS USE JUSTINA'S SOFTWARE..." << std::endl;
    ros::init(argc, argv, "test_path_planning");
    ros::NodeHandle n;
    ros::Rate loop(10);

    
    std::vector<float> goal_x, goal_y;
    
    JustinaNavigation::setNodeHandle(&n);
    //
    //Publishers and subscribers to set goal points, wait for the robot and count the collisions
    //ros::Publisher  pubGetClose    = n.advertise<std_msgs::Float32MultiArray>("/navigation/mvn_pln/get_close_xya", 10);
    ros::Subscriber subCollision   = n.subscribe("/navigation/obs_avoidance/collision_point", 10, callback_collision);
    //ros::Subscriber subGoalReached = n.subscribe("/navigation/global_goal_reached", 10, callback_goal_reached);

    //
    //Generation of random goal points and displaying in rviz as a line strip
    ros::Publisher pubRandomPoints = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);
    visualization_msgs::Marker mrkPoints;
    mrkPoints.header.frame_id = "map";
    mrkPoints.type = visualization_msgs::Marker::LINE_STRIP;
    mrkPoints.ns = "random_points";
    mrkPoints.id = 0;
    mrkPoints.action = visualization_msgs::Marker::ADD;
    mrkPoints.lifetime = ros::Duration();
    mrkPoints.color.r = 1.0;
    mrkPoints.color.g = 0.0;
    mrkPoints.color.b = 0.0;
    mrkPoints.color.a = 0.5;
    mrkPoints.scale.x = 0.03;
    mrkPoints.scale.y = 0.03;
    mrkPoints.scale.z = 0.03;

    mrkPoints.points.resize(goal_x.size() + 1);
    mrkPoints.points[0].x = 0;
    mrkPoints.points[0].y = 0;
    for(int i=0; i < goal_x.size(); i++)
    {
        mrkPoints.points[i+1].x = goal_x[i];
        mrkPoints.points[i+1].y = goal_y[i];
    }

    //
    //Variables for the state machine
    
    
    int collision_counter = 0;
    int current_state = SM_SEND_TO_GOAL;
    int current_goal = 0;

    while(ros::ok())
    {
        std_msgs::Float32MultiArray msgGetClose;
        switch(current_state)
        {
        case SM_SEND_TO_GOAL:
            std::cout << "Tester.->Sending robot to goal: " << goal_x[current_goal] << "  " << goal_y[current_goal] << std::endl;
            msgGetClose.data.resize(2);
            msgGetClose.data[0] = goal_x[current_goal];
            msgGetClose.data[1] = goal_y[current_goal];
            //pubGetClose.publish(msgGetClose);
            collision_counter = 0;
            JustinaNavigation::startGetClose(goal_x[current_goal], goal_y[current_goal]);
            current_state = SM_WAIT_FOR_GOAL_REACHED;
            break;
        case SM_WAIT_FOR_GOAL_REACHED:
            if(collision_detected)
            {
                collision_detected = false;
                collision_counter++;
            }
            if(JustinaNavigation::isGlobalGoalReached()) //Flag is changed in the callback
                current_state = SM_GOAL_REACHED;
            break;
        case SM_GOAL_REACHED:
            std::cout << "Tester.->Goal point reached with " << collision_counter << " collisions detected" << std::endl;
            current_goal++;
            if(current_goal >= goal_x.size())
            {
                std::cout << "Tester.->All points reached succesfully" << std::endl;
                current_state = -1;
            }
            else
                current_state = SM_SEND_TO_GOAL;
            break;
        default:
            break;
        }
        mrkPoints.header.stamp = ros::Time::now();
        pubRandomPoints.publish(mrkPoints);
	ros::spinOnce();
	loop.sleep();
    }
}
