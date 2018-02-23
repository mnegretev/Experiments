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
#include "visualization_msgs/Marker.h"
#include "justina_tools/JustinaNavigation.h"

#define SM_CHECK_STATUS 0
#define SM_SEND_TO_GOAL     10
#define SM_WAIT_FOR_GOAL_REACHED 20
#define SM_GOAL_REACHED   30
#define SM_SAVE_DATA  40

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

    float goal_point_x = 0;
    float goal_point_y = 0;
    bool correct_param_x = false;
    bool correct_param_y = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
	if(strParam.compare("-x") == 0 && (i + 1) < argc)
	{
	    std::stringstream ssX(argv[++i]);
	    if(ssX >> goal_point_x)
		correct_param_x = true;
	}
        if(strParam.compare("-y") == 0 && (i + 1) < argc)
	{
	    std::stringstream ssY(argv[++i]);
	    if(ssY >> goal_point_y)
		correct_param_y = true;
	}
    }
    if(!(correct_param_x && correct_param_y))
    {
	std::cout << "Too few parameters!!! Usage: " << std::endl;
	std::cout << "   -x goal_x -y goal_y" << std::endl;
	return 0;
    }

    std::cout << "Tester.-> Goal point: " << goal_point_x << "  " << goal_point_y << std::endl;
    
    std::vector<float> goal_x, goal_y;
    goal_x.push_back(goal_point_x);
    goal_y.push_back(goal_point_y);
    JustinaNavigation::setNodeHandle(&n);
    ros::Subscriber subCollision   = n.subscribe("/navigation/obs_avoidance/collision_point", 10, callback_collision);
    
    //
    //Variables for the state machine
    std::vector<int>   collisions;
    std::vector<int>   samples;
    std::vector<float> travel_distances;
    std::vector<float> travel_times;
    std::vector<float> theta_changes;
    collisions.resize(goal_x.size());
    samples.resize(goal_x.size());
    travel_distances.resize(goal_x.size());
    travel_times.resize(goal_x.size());
    theta_changes.resize(goal_x.size());
    
    int current_state = SM_SEND_TO_GOAL;
    int current_goal = 0;
    bool task_finished = false;
    float robot_x = 0, robot_y = 0, robot_t = 0;
    float last_robot_x = 0, last_robot_y = 0, last_robot_t = 0;
    ros::Time time_start, time_end;

    while(ros::ok() && ! task_finished)
    {
        last_robot_x = robot_x;
        last_robot_y = robot_y;
        last_robot_t = robot_t;
	JustinaNavigation::getRobotPose(robot_x, robot_y, robot_t);
	float delta_d = (robot_x - last_robot_x)*(robot_x - last_robot_x) + (robot_y - last_robot_y)*(robot_y - last_robot_y);
        float delta_t = fabs(robot_t - last_robot_t);
	delta_d = sqrt(delta_d);
	
        switch(current_state)
        {
	case SM_CHECK_STATUS:
	    if(current_goal >= goal_x.size())
            {
                std::cout << "Tester.->All points reached succesfully" << std::endl;
                current_state = SM_SAVE_DATA;
            }
            else
                current_state = SM_SEND_TO_GOAL;
	    break;
        case SM_SEND_TO_GOAL:
            std::cout << "Tester.->Sending robot to goal "<< current_goal << ":" << goal_x[current_goal] << "  " << goal_y[current_goal] << std::endl;
            collisions[current_goal] = 0;
	    travel_distances[current_goal] = 0;
            theta_changes[current_goal] = 0;
            samples[current_goal] = 0;
	    time_start = ros::Time::now();
            JustinaNavigation::startGetClose(goal_x[current_goal], goal_y[current_goal]);
            current_state = SM_WAIT_FOR_GOAL_REACHED;
            break;
        case SM_WAIT_FOR_GOAL_REACHED:
            if(collision_detected)
            {
                collision_detected = false;
                collisions[current_goal]++;
            }
	    travel_distances[current_goal] += delta_d;
            theta_changes[current_goal] += delta_t;
            samples[current_goal]++;
            if(JustinaNavigation::isGlobalGoalReached()) //Flag is changed in the callback
                current_state = SM_GOAL_REACHED;
            break;
        case SM_GOAL_REACHED:
	    time_end = ros::Time::now();
	    travel_times[current_goal] = (time_end - time_start).toSec();
            std::cout << "Tester.->Goal reached. Collisions: " << collisions[current_goal] << "\tDistance: ";
            std::cout << travel_distances[current_goal] << "\tTime: " << travel_times[current_goal] << "\tTheta changes: ";
            std::cout << theta_changes[current_goal] << "\tSamples: " << samples[current_goal] << std::endl;
            current_goal++;
	    current_state = SM_CHECK_STATUS;
            break;
        case SM_SAVE_DATA:
            task_finished = true;
            break;
        default:
            break;
        }
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
