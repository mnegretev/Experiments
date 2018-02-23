#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH PLOTTER NODE BY MARCOSOFT ..." << std::endl;
    ros::init(argc, argv, "path_plotter");
    ros::NodeHandle n;
    ros::Rate loop(10);
    
    tf::TransformListener listener;
    float robot_x = 0;
    float robot_y = 0;
    float robot_theta = 0;
    float last_robot_x = 0;
    float last_robot_y = 0;

    ros::Publisher pub_path = n.advertise<nav_msgs::Path>("/navigation/traveled_path", 1);
    nav_msgs::Path traveled_path;
    traveled_path.header.frame_id = "map";

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

	float dist = sqrt((robot_x - last_robot_x)*(robot_x - last_robot_x) + (robot_y - last_robot_y)*(robot_y - last_robot_y));
	if(dist > 0.05)
	{
	    geometry_msgs::PoseStamped p;
	    p.header.frame_id = "map";
	    p.pose.position.x = robot_x;
	    p.pose.position.y = robot_y;
	    traveled_path.poses.push_back(p);
	    last_robot_x = robot_x;
	    last_robot_y = robot_y;
	    std::cout << "Adding a new pose to the traveled path ..." << std::endl;
	}

	traveled_path.header.stamp = ros::Time::now();
	pub_path.publish(traveled_path);

	loop.sleep();
        ros::spinOnce();
    }
}
