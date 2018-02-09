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
    mrkPoints.type = visualization_msgs::Marker::SPHERE_LIST;
    mrkPoints.ns = "random_points";
    mrkPoints.id = 0;
    mrkPoints.action = visualization_msgs::Marker::ADD;
    mrkPoints.lifetime = ros::Duration();
    mrkPoints.color.r = 0.0;
    mrkPoints.color.g = 0.5;
    mrkPoints.color.b = 0.0;
    mrkPoints.color.a = 1.0;
    mrkPoints.scale.x = 0.1;
    mrkPoints.scale.y = 0.1;
    mrkPoints.scale.z = 0.1;

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
    std::cout << "INITIALIZING TEST OF PATH PLANNING ALGORITHMS USE JUSTINA'S SOFTWARE..." << std::endl;
    ros::init(argc, argv, "test_path_planning");
    ros::NodeHandle n;
    ros::Rate loop(10);

    std::string input_file = "";
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string output_file = boost::posix_time::to_iso_extended_string(my_posix_time);
    bool correct_params = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
	if(strParam.compare("--if") == 0 && (i + 1) < argc)
	{
	    correct_params = true;
	    input_file = argv[++i];
	}
        if(strParam.compare("--of") == 0 && (i + 1) < argc)
            output_file = argv[++i];
    }
    if(!correct_params)
    {
	std::cout << "Too few parameters!!! Usage: " << std::endl;
	std::cout << "   --if input_file [--of output_file]" << std::endl;
	std::cout << "   Input file is the file where the random points are stored." << std::endl;
	std::cout << "   This file can be generated using the random_goals_generator node" << std::endl;
        std::cout << "   Output file is the file where simulation result will be stored" << std::endl;
        std::cout << "   If no output file is specified, posix date and time is used" << std::endl;
	return 0;
    }

    std::string input_file_path = ros::package::getPath("experiments") + "/data/worlds/" + input_file + ".mpd";
    std::vector<float> goal_x, goal_y;
    visualization_msgs::Marker mrkPoints;
    if(!read_poses_from_file(goal_x, goal_y, input_file_path, mrkPoints))
    {
	std::cout << "Tester.-> Invalid data file :'(" << std::endl;
	return -1;
    }

    JustinaNavigation::setNodeHandle(&n);
    ros::Subscriber subCollision   = n.subscribe("/navigation/obs_avoidance/collision_point", 10, callback_collision);
    ros::Publisher pubRandomPoints = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);
    
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

    std::string output_file_path = ros::package::getPath("experiments") + "/data/results_" + output_file + ".mpd";
    std::ofstream output_stream;

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
            std::cout << "Tester.->Sending robot to goal: " << goal_x[current_goal] << "  " << goal_y[current_goal] << std::endl;
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
            std::cout << "Tester.->Saving data to file: " << output_file_path << std::endl;
            output_stream.open(output_file_path.c_str());
            output_stream << "Goal_X,\tGoal_Y,\tTime,\tDistance,\tTheta_Changes,\tCollisions,\tSamples" << std::endl;
            output_stream << std::fixed << std::setprecision(4);
            for(int i=0; i < goal_x.size(); i++)
            {
                output_stream << goal_x[i] << ",\t" << goal_y[i] << ",\t" << travel_times[i] << ",\t" << travel_distances[i];
                output_stream << ",\t" << theta_changes[i] << ",\t" << collisions[i] << ",\t" << samples[i] << std::endl;
            }
            output_stream.close();
            std::cout << "Tester.->Data saved to file: " << output_file_path << std::endl;
            task_finished = true;
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
