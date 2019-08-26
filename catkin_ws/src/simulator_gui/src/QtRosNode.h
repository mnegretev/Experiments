#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetPlan.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    bool gui_closed;
    bool continuous_mean_anomaly;
    float current_mean_anomaly;

private:
    ros::NodeHandle* n;

    ros::Publisher pub_joints;
    ros::Publisher pub_semimajor_axis;
    ros::Publisher pub_eccentricity;
    ros::Publisher pub_mean_anomaly;
    sensor_msgs::JointState msg_joints;

public:
    void run();
    void setNodeHandle(ros::NodeHandle* nh);
    
    void setAscendingNode      (float N);
    void setInclination        (float i);
    void setArgumentOfPeriapsis(float w);
    void publishSemimajorAxis  (float a);
    void publishEccentricity   (float e);
    void publishMeanAnomaly    (float M);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
