#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

void get_random_goal_poses(std::vector<float>& goal_x, std::vector<float>& goal_y, nav_msgs::OccupancyGrid& map,
                                 float map_x_min, float map_y_min, float map_x_max, float map_y_max, int number_of_points)
{
    int idx_x_min = int((map_x_min - map.info.origin.position.x) / map.info.resolution);
    int idx_x_max = int((map_x_max - map.info.origin.position.x) / map.info.resolution);
    int idx_y_min = int((map_y_min - map.info.origin.position.y) / map.info.resolution);
    int idx_y_max = int((map_y_max - map.info.origin.position.y) / map.info.resolution);
    goal_x.resize(number_of_points);
    goal_y.resize(number_of_points);
    
    srand(time(NULL));
    int idx_x;
    int idx_y;
    int attempts = 1000 * number_of_points;
    bool is_free_space = false;

    for(int i=0; i < number_of_points; i++)
    {
	is_free_space = false;
	while(!is_free_space && --attempts > 0)
	{
	    idx_x = rand()%(idx_x_max - idx_x_min) + idx_x_min;
	    idx_y = rand()%(idx_y_max - idx_y_min) + idx_y_min;
	    is_free_space = map.data[idx_y*map.info.width + idx_x] < 40 && map.data[idx_y*map.info.width + idx_x] >= 0;
	}
	if(is_free_space)
	{
	    goal_x[i] = idx_x * map.info.resolution + map.info.origin.position.x;
	    goal_y[i] = idx_y * map.info.resolution + map.info.origin.position.y;
	}
    }
}

void save_poses_to_file(std::vector<float>& goal_x, std::vector<float>& goal_y, nav_msgs::OccupancyGrid& map, float map_x_min,
			float map_y_min, float map_x_max, float map_y_max, int number_of_points, std::string file_path)
{
    //All data file are stored in package folder experiments/data
    std::ofstream goals_file;
    goals_file.open(file_path.c_str());
    goals_file << "//Random goal poses generated using the following info:" << std::endl;
    goals_file << "//Min X: " << map_x_min << "\tMin Y: " << map_y_min << "\tMax X: " << map_x_max << "\tMax Y: ";
    goals_file << map_y_max << "\tNumber of points: " << number_of_points << std::endl;
    goals_file << "//Map origin: " << map.info.origin.position.x << "  " << map.info.origin.position.y;
    goals_file << "\tResolution: " << map.info.resolution << std::endl;
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    goals_file << "//Time of generation: " << iso_time_str << std::endl;
    goals_file << "//By the way, the extension .mpd stands for motion planner data." << std::endl << std::endl;

    goals_file << std::fixed << std::setprecision(2);
    for(int i=0; i < goal_x.size(); i++)
	goals_file << goal_x[i] << ",\t\t" << goal_y[i] << std::endl;
    
    goals_file.close();
}

nav_msgs::OccupancyGrid grow_obstacles(nav_msgs::OccupancyGrid& map, float growDist)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    if(growDist <= 0)
    {
        std::cout << "PathCalculator.->Cannot grow map. Grow dist must be greater than zero." << std::endl;
        return map;
    }
    nav_msgs::OccupancyGrid newMap = map;
    
    int growSteps = (int)(growDist / map.info.resolution);
    int boxSize = (2*growSteps + 1) * (2*growSteps + 1);
    int* neighbors = new int[boxSize];
    int counter = 0;

    //std::cout << "PathCalculator.->Growing map " << growSteps << " steps" << std::endl;
    for(int i=-growSteps; i<=growSteps; i++)
        for(int j=-growSteps; j<=growSteps; j++)
        {
            neighbors[counter] = j*map.info.width + i;
            counter++;
        }
    /*
    std::cout << "Calculation of neighbors finished: " << std::endl;
    for(int i=0; i <boxSize; i++)
        std::cout << neighbors[i] << std::endl;
    */
    int startIdx = growSteps*map.info.width + growSteps;
    int endIdx = map.data.size() - growSteps*map.info.width - growSteps;

    if(endIdx <= 0)
    {
        std::cout << "PathCalculator.->Cannot grow map. Map is smaller than desired growth." << std::endl;
        return map;
    }

    for(int i=startIdx; i < endIdx; i++)
        if(map.data[i] > 40 || map.data[i] < 0) //Then, is an occupied or unknown cell
            for(int j=0; j < boxSize; j++) //If it is occupied, mark as occupied all neighbors in the neighbor-box
                newMap.data[i+neighbors[j]] = 100;

    delete[] neighbors;
    //std::cout << "PathCalculator.->Map-growth finished." << std::endl;
    return newMap;
}

int main(int argc, char** argv)
{	
    std::cout << "INITIALIZING RANDOM GOAL POSE GENERATOR..." << std::endl;
    ros::init(argc, argv, "random_goals_generator");
    ros::NodeHandle n;

    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string file_name = boost::posix_time::to_iso_extended_string(my_posix_time);
    
    float min_x = 0, min_y = 0, max_x = 10, max_y = 10;
    int number_of_points = 1000;
    bool correct_params = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
	    file_name = argv[++i];
	if(strParam.compare("-n") == 0)
	{
	    std::stringstream ss_n(argv[++i]);
	    int val;
	    if(ss_n >> val)
		number_of_points = val;
	}
	if(strParam.compare("--limits") == 0 && (i + 4) <= argc)
	{
	    correct_params = true;
	    std::stringstream ss_min_x(argv[++i]);
	    correct_params &= (ss_min_x >> min_x) != 0;
	    std::stringstream ss_min_y(argv[++i]);
	    correct_params &= (ss_min_y >> min_y) != 0;
	    std::stringstream ss_max_x(argv[++i]);
	    correct_params &= (ss_max_x >> max_x) != 0;
	    std::stringstream ss_max_y(argv[++i]);
	    correct_params &= (ss_max_y >> max_y) != 0;
	}
    }

    if(!correct_params)
    {
	std::cout << "Too few parameters!!! Usage: " << std::endl;
	std::cout << "   --limits min_x min_y max_x max_y  [-f file_name] [-n number_of_points]" << std::endl;
	std::cout << "   Default file name is the posix date and time" << std::endl;
	std::cout << "   Default number of points is 1000" << std::endl;
	return 0;
    }

    std::cout << "Waiting for the service to get a static map ..." << std::endl;
    if(!ros::service::waitForService("/navigation/localization/static_map"))
    {
	std::cout << "Cannot get a static map. It is needed to generate valid goal points. Sorry" << std::endl;
	return -1;
    }

    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    nav_msgs::GetMap srvGetMap;
    srvCltGetMap.call(srvGetMap);
    nav_msgs::OccupancyGrid map = srvGetMap.response.map;
    map = grow_obstacles(map, 0.65);

    std::string file_path = ros::package::getPath("experiments") + "/data/" + file_name + ".mpd";
    
    std::vector<float> goal_x, goal_y;
    std::cout << "Generating random goal points..." << std::endl;
    get_random_goal_poses(goal_x, goal_y, map, min_x, min_y, max_x, max_y, number_of_points);
    save_poses_to_file(goal_x, goal_y, map, min_x, min_y, max_x, max_y, number_of_points, file_path);
    
    std::cout << "Random goal points generated succesfully and saved to file " << file_name << std::endl;
    return 0;
}
