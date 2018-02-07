#include <ros/ros.h>
#include <wrl_to_map/BresenhamAlgorithm.h>
#include <wrl_to_map/parserFileWRL.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char ** argv)
{
    std::string input_file = "";
    bool correct_params = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
	if(strParam.compare("-f") == 0 && (i + 1) < argc)
	{
	    correct_params = true;
	    input_file = argv[++i];
	}
    }
    if(!correct_params)
    {
	std::cout << "Too few parameters!!! Usage: " << std::endl;
	std::cout << "wrl_to_mapa -f input_file" << std::endl;
	std::cout << "Input file is the file with the world info in savage format" << std::endl;
	return 0;
    }

    ros::init(argc, argv, "wrl_to_map_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    std::cout << "Parsing file: " << input_file << std::endl;
    std::vector<biorobotics::Polygon> polygons = biorobotics::parserFile(input_file);

    ros::Publisher pubMap = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

    biorobotics::Vertex2 origin(-1, -1);
    float res = 0.05;
    float w = 17, h = 17;
    int sizeOccupancyMatrix = (int) (w / res * h / res);
    
    signed char * occupancyMatrix = new signed char[sizeOccupancyMatrix];
    int cols = (int) (w / res);
    int rows = (int) (h / res);
    for(int i = 0; i < cols; i++)
        for(int j = 0; j < rows; j++){
            occupancyMatrix[i + j * cols] = 0;
        }

    std::cout << "Generating occupancy grid from parsed file" << std::endl;
    for(int i = 0 ; i < polygons.size(); i++){
        biorobotics::Polygon polygon = polygons[i];
        for(int j = 0; j < polygon.num_vertex; j++){
            biorobotics::Vertex2 v1 = polygon.vertex[j];
            biorobotics::Vertex2 v2;
            if(j < polygon.num_vertex - 1)
                v2 = polygon.vertex[j + 1];
            else
                v2 = polygon.vertex[0];
            int cellX1 = (int)((v1.x - origin.x) / res);
            int cellY1 = (int)((v1.y - origin.y) / res);
            int cellX2 = (int)((v2.x - origin.x) / res);
            int cellY2 = (int)((v2.y - origin.y) / res);
            bresenhamAlgorithm(cellX1, cellY1, cellX2, cellY2, occupancyMatrix, cols, rows, res);
        }
    }

    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "/map";
    int seq = 0;
    grid.info.width = cols;
    grid.info.height = rows;
    grid.info.resolution = 0.05;
    grid.info.origin.position.x = origin.x;
    grid.info.origin.position.y = origin.y;
    grid.data.insert(grid.data.begin(), occupancyMatrix, occupancyMatrix + sizeOccupancyMatrix);

    std::cout << "File parsed and map generated succesfully" << std::endl;
    while(ros::ok()){

        grid.header.stamp = ros::Time();
        grid.header.seq = seq++;

        pubMap.publish(grid);

        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
