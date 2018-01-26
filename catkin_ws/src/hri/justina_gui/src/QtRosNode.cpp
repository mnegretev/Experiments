#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    subGoalPointPose = n->subscribe("/move_base_simple/goal", 1, &QtRosNode::callback_goal_pose, this);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        emit updateGraphics();
        loop.sleep();
        ros::spinOnce();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
    JustinaHardware::setNodeHandle(nh);
    JustinaNavigation::setNodeHandle(nh);
    JustinaHRI::setNodeHandle(nh);
    JustinaManip::setNodeHandle(nh);
    JustinaVision::setNodeHandle(nh);
    JustinaTools::setNodeHandle(nh);
    JustinaKnowledge::setNodeHandle(nh);
    JustinaRepresentation::setNodeHandle(nh);
}

void QtRosNode::callback_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float theta = atan2(msg->pose.orientation.z, msg->pose.orientation.w)*2;
    emit onGoalPoseReceived(x, y, theta);
}
