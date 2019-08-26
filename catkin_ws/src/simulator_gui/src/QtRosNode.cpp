#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    this->continuous_mean_anomaly = false;
    this->current_mean_anomaly = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{
    pub_joints = n->advertise<sensor_msgs::JointState>("/joint_states", 1);
    pub_semimajor_axis = n->advertise<std_msgs::Float32>("/semimajor_axis", 1);
    pub_eccentricity   = n->advertise<std_msgs::Float32>("/eccentricity", 1);
    pub_mean_anomaly   = n->advertise<std_msgs::Float32>("/mean_anomaly", 1);
    msg_joints.name.resize(3);
    msg_joints.name[0] = "ascending_node_longitude";
    msg_joints.name[1] = "inclination";
    msg_joints.name[2] = "argument_of_periapsis";
    msg_joints.position.resize(3);
    std_msgs::Float32 msg_mean_anomaly;
    ros::Rate loop(50);

    while(ros::ok() && !this->gui_closed)
    {
        msg_joints.header.stamp = ros::Time::now();
        pub_joints.publish(msg_joints);
        if(continuous_mean_anomaly)
        {
            current_mean_anomaly += 0.03;
            if(current_mean_anomaly >= 2*M_PI)
                current_mean_anomaly-=2*M_PI;
            msg_mean_anomaly.data = current_mean_anomaly;
            pub_mean_anomaly.publish(msg_mean_anomaly);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::setAscendingNode(float N)
{
    msg_joints.position[0] = N;
}

void QtRosNode::setInclination(float i)
{
    msg_joints.position[1] = i;
}

void QtRosNode::setArgumentOfPeriapsis(float w)
{
    msg_joints.position[2] = w;
}

void QtRosNode::publishSemimajorAxis(float a)
{
    std_msgs::Float32 msg;
    msg.data = a;
    pub_semimajor_axis.publish(msg);
}

void QtRosNode::publishEccentricity(float e)
{
    std_msgs::Float32 msg;
    msg.data = e;
    pub_eccentricity.publish(msg);
}

void QtRosNode::publishMeanAnomaly(float M)
{
    std_msgs::Float32 msg;
    msg.data = M;
    pub_mean_anomaly.publish(msg);
}
