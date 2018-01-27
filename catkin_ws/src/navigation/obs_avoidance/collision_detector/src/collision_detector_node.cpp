#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

void get_offets_around_robot(std::vector<int>& offsets, int map_width)
{
    //Calculates a set of offset index of those cells around the robot
    //Considering the robot is a square of 0.5x0.5 m
    offsets.clear();
    offsets.resize(100);
    int idx = 0;
    for(int i=-5; i <5; i++)
	for(int j=-5; j< 5; j++, idx++)
	    offsets[idx] = j*map_width + i;
    
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING COLLISION DETECTOR NODE BY MARCOSOFT ..." << std::endl;
    ros::init(argc, argv, "collision_detector");
    ros::NodeHandle n;
    ros::Rate loop(10);

    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    nav_msgs::OccupancyGrid map = srvGetMap.response.map;
    tf::TransformListener listener;

    float robot_x = 0;
    float robot_y = 0;
    float robot_theta = 0;
    std::vector<int> offsets;
    get_offets_around_robot(offsets, map.info.width);
    bool collision = false;
    bool last_collision = false;

    ros::Publisher pubCollisionPoint = n.advertise<geometry_msgs::PointStamped>("collision_point", 1);
    geometry_msgs::PointStamped collisionPoint;
    collisionPoint.header.frame_id = "map";
    collisionPoint.point.z = 0.2;
    
    while(ros::ok())
    {
	tf::StampedTransform transform;
	tf::Quaternion q;
	try
	{
	    listener.lookupTransform("map", "base_link", ros::Time(0), transform);
	    robot_x = transform.getOrigin().x();
	    robot_y = transform.getOrigin().y();
	    q = transform.getRotation();
	    robot_theta = atan2(q.z(), q.w())*2;
	}
	catch(...){std::cout << "Collision detector.-> Cannot get transform from base_link to map" << std::endl;}

	int idx = int((robot_y - map.info.origin.position.y)/map.info.resolution)*map.info.width;
	idx += int((robot_x - map.info.origin.position.x)/map.info.resolution);

	collision = false;
	int i = 0;
	for(i=0; i < offsets.size() && !collision; i++)
	    collision |= map.data[idx + offsets[i]] > 50;
	i--;
	
	if(collision && !last_collision)
	{
	    std::cout << "CollisionDetector.-> Collision detected between robot and static map" << std::endl;
	    collisionPoint.point.x = ((idx + offsets[i]) % map.info.width)*map.info.resolution + map.info.origin.position.x;
	    collisionPoint.point.y = ((idx + offsets[i]) / map.info.width)*map.info.resolution + map.info.origin.position.y;
	    collisionPoint.header.stamp = ros::Time::now();
	    pubCollisionPoint.publish(collisionPoint);
	}

	last_collision = collision;

	loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
