#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"

void get_random_start_goal_poses(std::vector<float>& start_x, std::vector<float>& start_y, std::vector<float>& goal_x,
				 std::vector<float>& goal_y, nav_msgs::OccupancyGrid& map, float map_x_min, float map_x_max,
				 float map_y_min, float map_y_max, int number_of_pairs)
{
    int idx_x_min = int((map_x_min - map.info.origin.position.x) / map.info.resolution);
    int idx_x_max = int((map_x_max - map.info.origin.position.x) / map.info.resolution);
    int idx_y_min = int((map_y_min - map.info.origin.position.y) / map.info.resolution);
    int idx_y_max = int((map_y_max - map.info.origin.position.y) / map.info.resolution);
    start_x.resize(number_of_pairs);
    start_y.resize(number_of_pairs);
    goal_x.resize(number_of_pairs);
    goal_y.resize(number_of_pairs);
    
    srand(time(NULL));
    int idx_x;
    int idx_y;
    int attempts = 1000 * number_of_pairs;
    bool is_free_space = false;
    for(int i=0; i < number_of_pairs; i++)
    {
	is_free_space = false;
	while(!is_free_space && --attempts > 0)
	{
	    idx_x = rand()%(idx_x_max - idx_x_min) + idx_x_min;
	    idx_y = rand()%(idx_y_max - idx_y_min) + idx_y_min;
	    is_free_space = map.data[idx_y*map.info.width + idx_x] < 40;
	}
	if(is_free_space)
	{
	    start_x[i] = idx_x * map.info.resolution + map.info.origin.position.x;
	    start_y[i] = idx_y * map.info.resolution + map.info.origin.position.y;
	}
    }
    for(int i=0; i < number_of_pairs; i++)
    {
	is_free_space = false;
	while(!is_free_space && --attempts > 0)
	{
	    idx_x = rand()%(idx_x_max - idx_x_min) + idx_x_min;
	    idx_y = rand()%(idx_y_max - idx_y_min) + idx_y_min;
	    is_free_space = map.data[idx_y*map.info.width + idx_x] < 40;
	}
	if(is_free_space)
	{
	    goal_x[i] = idx_x * map.info.resolution + map.info.origin.position.x;
	    goal_y[i] = idx_y * map.info.resolution + map.info.origin.position.y;
	}
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING TEST OF PATH PLANNING ALGORITHMS USE JUSTINA'S SOFTWARE..." << std::endl;
    ros::init(argc, argv, "test_path_planning");
    ros::NodeHandle n;
    ros::Rate loop(10);

    while(ros::ok())
    {
	ros::spinOnce();
	loop.sleep();
    }
}
