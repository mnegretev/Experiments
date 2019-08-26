#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

def calculate_eccentric_anomaly(M,e):
    E0 = M
    E1 = E0 - (M + e*math.sin(E0) - E0)/(e*math.cos(E0)-1)
    timeout = 100
    while abs(E1-E0) > 0.0000005 and timeout > 0:
        E0 = E1
        E1 = E0 - (M + e*math.sin(E0) - E0)/(e*math.cos(E0)-1)
        timeout -= 1
    if timeout == 0:
        print("WARNING! Eccentric anomaly did not converge.")
    return E1

def callback_semimajor_axis(msg):
    global semimajor_axis
    semimajor_axis = msg.data

def callback_eccentricity(msg):
    global eccentricity
    eccentricity = msg.data

def callback_mean_anomaly(msg):
    global mean_anomaly
    mean_anomaly = msg.data

def main():
    global eccentricity
    global semimajor_axis
    global mean_anomaly
    eccentricity   = 0.5
    semimajor_axis = 2.5
    mean_anomaly   = 0
    
    rospy.init_node('planet_orbit')
    pub_planet   = rospy.Publisher('planet', PointStamped, queue_size=1)
    pub_planet_M = rospy.Publisher('planet_circle', PointStamped, queue_size=1)
    pub_orbit_elements = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    sub_semimajor_axis = rospy.Subscriber('semimajor_axis', Float32, callback_semimajor_axis)
    sub_eccentricity   = rospy.Subscriber('eccentricity'  , Float32, callback_eccentricity)
    sub_mean_anomaly   = rospy.Subscriber('mean_anomaly'  , Float32, callback_mean_anomaly)
    
    msg_planet = PointStamped()
    msg_planet.header.frame_id = 'sun_w'
    msg_planet.point.z = 0
    
    msg_planet_M = PointStamped()
    msg_planet_M.header.frame_id = 'sun_w'
    msg_planet_M.point.z = 0
    
    msg_orbit = Marker()
    msg_orbit.header.frame_id = 'sun_w'
    msg_orbit.ns              = 'planet_system'
    msg_orbit.id              = 0
    msg_orbit.type            = Marker.CYLINDER
    msg_orbit.action          = Marker.ADD
    msg_orbit.scale.z = 0.02
    msg_orbit.color.r = 0.4
    msg_orbit.color.g = 0.4
    msg_orbit.color.b = 0.6
    msg_orbit.color.a = 1.0
    msg_orbit.pose.position.y = 0
    msg_orbit.pose.position.z = 0
    
    msg_orbit_circle = Marker()
    msg_orbit_circle.header.frame_id = 'sun_w'
    msg_orbit_circle.ns              = 'planet_system'
    msg_orbit_circle.id              = 1
    msg_orbit_circle.type            = Marker.CYLINDER
    msg_orbit_circle.action          = Marker.ADD
    msg_orbit_circle.scale.z = 0.01
    msg_orbit_circle.color.r = 0.6
    msg_orbit_circle.color.g = 0.6
    msg_orbit_circle.color.b = 0.8
    msg_orbit_circle.color.a = 1.0
    msg_orbit_circle.pose.position.y = 0
    msg_orbit_circle.pose.position.z = 0

    msg_ascending_node = Marker()
    msg_ascending_node.header.frame_id = 'sun_N'
    msg_ascending_node.ns              = 'planet_system'
    msg_ascending_node.id              = 2
    msg_ascending_node.type            = Marker.ARROW
    msg_ascending_node.action          = Marker.ADD
    msg_ascending_node.points.append(Point())
    msg_ascending_node.points.append(Point())
    msg_ascending_node.points[0].x = 0
    msg_ascending_node.points[0].y = 0
    msg_ascending_node.points[0].z = 0
    msg_ascending_node.points[1].y = 0
    msg_ascending_node.points[1].z = 0
    msg_ascending_node.scale.x = 0.04
    msg_ascending_node.scale.y = 0.1
    msg_ascending_node.scale.z = 0
    msg_ascending_node.color.r = 1.0
    msg_ascending_node.color.g = 0.0
    msg_ascending_node.color.b = 0.0
    msg_ascending_node.color.a = 1.0

    msg_periapsis = Marker()
    msg_periapsis.header.frame_id = 'sun_w'
    msg_periapsis.ns              = 'planet_system'
    msg_periapsis.id              = 3
    msg_periapsis.type            = Marker.ARROW
    msg_periapsis.action          = Marker.ADD
    msg_periapsis.points.append(Point())
    msg_periapsis.points.append(Point())
    msg_periapsis.points[0].y = 0
    msg_periapsis.points[0].z = 0
    msg_periapsis.points[1].y = 0
    msg_periapsis.points[1].z = 0
    msg_periapsis.scale.x = 0.04
    msg_periapsis.scale.y = 0.1
    msg_periapsis.scale.z = 0
    msg_periapsis.color.r = 1.0
    msg_periapsis.color.g = 0.0
    msg_periapsis.color.b = 0.0
    msg_periapsis.color.a = 1.0

    msg_mean_anomaly = Marker()
    msg_mean_anomaly.header.frame_id = 'sun_w'
    msg_mean_anomaly.ns              = 'anomalies'
    msg_mean_anomaly.id              = 4
    msg_mean_anomaly.type            = Marker.ARROW
    msg_mean_anomaly.action          = Marker.ADD
    msg_mean_anomaly.points.append(Point())
    msg_mean_anomaly.points.append(Point())
    msg_mean_anomaly.points[0].y = 0
    msg_mean_anomaly.points[0].z = 0
    msg_mean_anomaly.points[1].z = 0
    msg_mean_anomaly.scale.x = 0.04
    msg_mean_anomaly.scale.y = 0.1
    msg_mean_anomaly.scale.z = 0
    msg_mean_anomaly.color.r = 0.0
    msg_mean_anomaly.color.g = 1.0
    msg_mean_anomaly.color.b = 0.5
    msg_mean_anomaly.color.a = 1.0

    msg_eccentric_anomaly = Marker()
    msg_eccentric_anomaly.header.frame_id = 'sun_w'
    msg_eccentric_anomaly.ns              = 'anomalies'
    msg_eccentric_anomaly.id              = 5
    msg_eccentric_anomaly.type            = Marker.ARROW
    msg_eccentric_anomaly.action          = Marker.ADD
    msg_eccentric_anomaly.points.append(Point())
    msg_eccentric_anomaly.points.append(Point())
    msg_eccentric_anomaly.points[0].y = 0
    msg_eccentric_anomaly.points[0].z = 0
    msg_eccentric_anomaly.points[1].z = 0
    msg_eccentric_anomaly.scale.x = 0.04
    msg_eccentric_anomaly.scale.y = 0.1
    msg_eccentric_anomaly.scale.z = 0
    msg_eccentric_anomaly.color.r = 0.0
    msg_eccentric_anomaly.color.g = 0.6
    msg_eccentric_anomaly.color.b = 0.0
    msg_eccentric_anomaly.color.a = 0.7
    
    msg_true_anomaly = Marker()
    msg_true_anomaly.header.frame_id = 'sun_w'
    msg_true_anomaly.ns              = 'anomalies'
    msg_true_anomaly.id              = 6
    msg_true_anomaly.type            = Marker.ARROW
    msg_true_anomaly.action          = Marker.ADD
    msg_true_anomaly.points.append(Point())
    msg_true_anomaly.points.append(Point())
    msg_true_anomaly.points[0].x = 0
    msg_true_anomaly.points[0].y = 0
    msg_true_anomaly.points[0].z = 0
    msg_true_anomaly.points[1].z = 0
    msg_true_anomaly.scale.x = 0.04
    msg_true_anomaly.scale.y = 0.1
    msg_true_anomaly.scale.z = 0
    msg_true_anomaly.color.r = 0.0
    msg_true_anomaly.color.g = 1.0
    msg_true_anomaly.color.b = 0.0
    msg_true_anomaly.color.a = 1.0

    loop = rospy.Rate(50)

    while not rospy.is_shutdown():
        eccentric_anomaly = calculate_eccentric_anomaly(mean_anomaly, eccentricity)
        planet_x = semimajor_axis*(math.cos(eccentric_anomaly) - eccentricity)
        planet_y = semimajor_axis*math.sqrt(1-eccentricity*eccentricity)*math.sin(eccentric_anomaly)
        mean_anomaly_x = semimajor_axis*math.cos(mean_anomaly) - semimajor_axis*eccentricity
        mean_anomaly_y = semimajor_axis*math.sin(mean_anomaly)
        ellipse_center_x = -semimajor_axis*eccentricity
        ellipse_max_x    = semimajor_axis*(1-eccentricity)

        msg_planet.point.x = planet_x
        msg_planet.point.y = planet_y

        msg_planet_M.point.x = mean_anomaly_x
        msg_planet_M.point.y = mean_anomaly_y

        msg_orbit.scale.x = semimajor_axis*2
        msg_orbit.scale.y = math.sqrt(1-eccentricity*eccentricity)*semimajor_axis*2
        msg_orbit.pose.position.x = ellipse_center_x
        
        msg_orbit_circle.scale.x = semimajor_axis*2
        msg_orbit_circle.scale.y = semimajor_axis*2
        msg_orbit_circle.pose.position.x = ellipse_center_x

        msg_ascending_node.points[1].x = ellipse_max_x
        msg_periapsis.points[0].x      = ellipse_center_x
        msg_periapsis.points[1].x      = ellipse_max_x
        msg_mean_anomaly.points[0].x   = ellipse_center_x
        msg_mean_anomaly.points[1].x   = mean_anomaly_x
        msg_mean_anomaly.points[1].y   = mean_anomaly_y
        msg_eccentric_anomaly.points[0].x   = ellipse_center_x
        msg_eccentric_anomaly.points[1].x   = planet_x
        msg_eccentric_anomaly.points[1].y   = planet_y
        msg_true_anomaly.points[1].x   = planet_x
        msg_true_anomaly.points[1].y   = planet_y

        msg_planet.header.stamp = rospy.Time.now()
        msg_planet_M.header.stamp = rospy.Time.now()
        msg_orbit.header.stamp = rospy.Time.now()
        msg_orbit_circle.header.stamp = rospy.Time.now()
        msg_ascending_node.header.stamp = rospy.Time.now()
        msg_periapsis.header.stamp = rospy.Time.now()
        msg_mean_anomaly.header.stamp = rospy.Time.now()
        msg_eccentric_anomaly.header.stamp = rospy.Time.now()
        msg_true_anomaly.header.stamp = rospy.Time.now()
        pub_planet.publish(msg_planet)
        pub_planet_M.publish(msg_planet_M)
        pub_orbit_elements.publish(msg_orbit)
        pub_orbit_elements.publish(msg_orbit_circle)
        pub_orbit_elements.publish(msg_ascending_node)
        pub_orbit_elements.publish(msg_periapsis)
        pub_orbit_elements.publish(msg_mean_anomaly)
        pub_orbit_elements.publish(msg_eccentric_anomaly)
        pub_orbit_elements.publish(msg_true_anomaly)
        loop.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
