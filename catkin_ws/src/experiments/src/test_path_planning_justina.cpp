#include "ros/ros.h"
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

void get_random_start_goal_poses(std::vector<float>& goal_x, std::vector<float>& goal_y, nav_msgs::OccupancyGrid& map,
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
        if(map.data[i] > 40) //Then, is an occupied cell
            for(int j=0; j < boxSize; j++) //If it is occupied, mark as occupied all neighbors in the neighbor-box
                newMap.data[i+neighbors[j]] = 100;

    delete[] neighbors;
    //std::cout << "PathCalculator.->Map-growth finished." << std::endl;
    return newMap;
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

    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    nav_msgs::OccupancyGrid map = srvGetMap.response.map;
    map = grow_obstacles(map, 0.25);

    std::vector<float> goal_x, goal_y;
    get_random_start_goal_poses(goal_x, goal_y, map, -2, -2, 8.0, 12.0, 1000);
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
