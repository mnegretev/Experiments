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

#define SM_CHECK_STATUS 0
#define SM_SEND_TO_GOAL     10
#define SM_WAIT_FOR_GOAL_REACHED 20
#define SM_GOAL_REACHED   30

bool collision_detected = false;
float collision_x = 0;
float collision_y = 0;
bool global_goal_reached = false;

bool read_poses_from_file(std::vector<float>& goal_x, std::vector<float>& goal_y, std::string file_path,
			  visualization_msgs::Marker& marker)
{
    std::cout << "Tester.->Loading known locations from " << file_path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(file_path.c_str());
    std::string tempStr;
    while (std::getline(file, tempStr))
        lines.push_back(tempStr);
    file.close();

    //Extraction of lines without comments
    for (size_t i = 0; i < lines.size(); i++)
    {
        size_t idx = lines[i].find("//");
        if (idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    float x, y;
    goal_x.clear();
    goal_y.clear();
    for (size_t i = 0; i < lines.size(); i++)
    {
        std::vector<std::string> parts;
        std::vector<float> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if (parts.size() < 2) //Only takes those lines with two or more values
            continue;
	std::stringstream ssX(parts[0]);
	std::stringstream ssY(parts[1]);
	if(ssX >> x && ssY >> y)
	{
	    goal_x.push_back(x);
	    goal_y.push_back(y);
	}
    }
    std::cout << "Tester.->Total of parsed points: " << goal_x.size() << std::endl;
    if(goal_x.size() < 1)
	return false;
    
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
    marker = mrkPoints;
    return true;
}

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
    std::string file_name = "";
    bool correct_params = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
	if(strParam.compare("-f") == 0 && (i + 1) < argc)
	{
	    correct_params = true;
	    file_name = argv[++i];
	}
    }
    if(!correct_params)
    {
	std::cout << "Too few parameters!!! Usage: " << std::endl;
	std::cout << "   -f file_name" << std::endl;
	std::cout << "   File name where the random points are stored." << std::endl;
	std::cout << "   This file can be generated using the random_goals_generator node" << std::endl;
	return 0;
    }
    
    std::cout << "INITIALIZING TEST OF PATH PLANNING ALGORITHMS USE JUSTINA'S SOFTWARE..." << std::endl;
    ros::init(argc, argv, "test_path_planning");
    ros::NodeHandle n;
    ros::Rate loop(10);

    std::string file_path = ros::package::getPath("experiments") + "/data/" + file_name + ".mpd";
    std::vector<float> goal_x, goal_y;
    visualization_msgs::Marker mrkPoints;
    if(!read_poses_from_file(goal_x, goal_y, file_path, mrkPoints))
    {
	std::cout << "Tester.-> Invalid data file :'(" << std::endl;
	return -1;
    }

    JustinaNavigation::setNodeHandle(&n);
    ros::Subscriber subCollision   = n.subscribe("/navigation/obs_avoidance/collision_point", 10, callback_collision);
    ros::Publisher pubRandomPoints = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);
    
    //
    //Variables for the state machine
    int collision_counter = 0;
    int current_state = SM_SEND_TO_GOAL;
    int current_goal = 0;
    bool task_finished = false;
    float robot_x = 0, robot_y = 0, robot_t = 0;
    float last_robot_x = 0, last_robot_y = 0, last_robot_t = 0;
    float traveled_dist = 0;
    double traveling_time = 0;
    ros::Time time_start, time_end;

    while(ros::ok() && ! task_finished)
    {
	JustinaNavigation::getRobotPose(robot_x, robot_y, robot_t);
	float delta_d = (robot_x - last_robot_x)*(robot_x - last_robot_x) + (robot_y - last_robot_y)*(robot_y - last_robot_y);
	delta_d = sqrt(delta_d);
	
        switch(current_state)
        {
	case SM_CHECK_STATUS:
	    if(current_goal >= goal_x.size())
            {
                std::cout << "Tester.->All points reached succesfully" << std::endl;
                task_finished = true;
            }
            else
                current_state = SM_SEND_TO_GOAL;
	    break;
        case SM_SEND_TO_GOAL:
            std::cout << "Tester.->Sending robot to goal: " << goal_x[current_goal] << "  " << goal_y[current_goal] << std::endl;
            collision_counter = 0;
	    traveled_dist = 0;
	    time_start = ros::Time::now();
            JustinaNavigation::startGetClose(goal_x[current_goal], goal_y[current_goal]);
            current_state = SM_WAIT_FOR_GOAL_REACHED;
            break;
        case SM_WAIT_FOR_GOAL_REACHED:
            if(collision_detected)
            {
                collision_detected = false;
                collision_counter++;
            }
	    traveled_dist += delta_d;
            if(JustinaNavigation::isGlobalGoalReached()) //Flag is changed in the callback
                current_state = SM_GOAL_REACHED;
            break;
        case SM_GOAL_REACHED:
	    time_end = ros::Time::now();
            std::cout << "Tester.->Goal point reached with: " << std::endl;
	    std::cout << "Collisions: " << collision_counter << "\tDistance: " << traveled_dist;
	    std::cout << "Time: " << traveling_time << std::endl;
            current_goal++;
	    traveling_time = (time_end - time_start).toSec();
	    current_state = SM_CHECK_STATUS;
            break;
        default:
            break;
        }
        mrkPoints.header.stamp = ros::Time::now();
        pubRandomPoints.publish(mrkPoints);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
