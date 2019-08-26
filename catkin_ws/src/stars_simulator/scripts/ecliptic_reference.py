#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

def main():
    rospy.init_node('orbital_params')
    pub_sun = rospy.Publisher('/sun', PointStamped, queue_size=1)
    pub_fpa = rospy.Publisher('visualization_marker', Marker, queue_size=1)
    msg_sun = PointStamped()
    msg_fpa = Marker() #Message to publish an arrow indicating the first point of Aries
    msg_sun.header.frame_id = 'sun_ecliptic'
    msg_fpa.header.frame_id = 'sun_ecliptic'
    msg_fpa.ns              = 'ecliptic_system'
    msg_fpa.id              = 0
    msg_fpa.type            = Marker.ARROW
    msg_fpa.action          = Marker.ADD
    msg_fpa.points.append(Point())
    msg_fpa.points.append(Point())
    msg_fpa.points[0].x = 0
    msg_fpa.points[0].y = 0
    msg_fpa.points[0].z = 0
    msg_fpa.points[1].x = 3
    msg_fpa.points[1].y = 0
    msg_fpa.points[1].z = 0
    msg_fpa.scale.x = 0.04
    msg_fpa.scale.y = 0.1
    msg_fpa.scale.z = 0
    msg_fpa.color.r = 1.0
    msg_fpa.color.g = 0.0
    msg_fpa.color.b = 0.0
    msg_fpa.color.a = 0.7
    msg_txt = Marker()
    msg_txt.header.frame_id = 'sun_ecliptic'
    msg_txt.ns              = 'ecliptic_texts'
    msg_txt.id              = 1
    msg_txt.type            = Marker.TEXT_VIEW_FACING
    msg_txt.action          = Marker.ADD
    msg_txt.text            = "First point of Aries"
    msg_txt.color.r = 1.0
    msg_txt.color.g = 1.0
    msg_txt.color.b = 1.0
    msg_txt.color.a = 1.0
    msg_txt.scale.z = 0.15
    msg_txt.pose.position.x = 3.4
    msg_txt.pose.position.y = 0.1
    msg_txt.pose.position.z = 0
    eccentricity = 0.5
    semimajor    = 2.5
    msg_ecl = Marker()
    msg_ecl.header.frame_id = 'sun_ecliptic'
    msg_ecl.ns              = 'ecliptic_system'
    msg_ecl.id              = 2
    msg_ecl.type            = Marker.CYLINDER
    msg_ecl.action          = Marker.ADD
    msg_ecl.scale.x = semimajor*2
    msg_ecl.scale.y = math.sqrt(1-eccentricity*eccentricity)*semimajor*2
    msg_ecl.scale.z = 0.02
    msg_ecl.color.r = 0.5
    msg_ecl.color.g = 0.5
    msg_ecl.color.b = 0.5
    msg_ecl.color.a = 1.0
    msg_ecl.pose.position.x = -semimajor*eccentricity
    msg_ecl.pose.position.y = 0
    msg_ecl.pose.position.z = 0
    
    loop = rospy.Rate(30)

    while not rospy.is_shutdown():
        msg_sun.header.stamp = rospy.Time.now()
        msg_fpa.header.stamp = rospy.Time.now()
        msg_txt.header.stamp = rospy.Time.now()
        msg_ecl.header.stamp = rospy.Time.now()
        pub_sun.publish(msg_sun)
        pub_fpa.publish(msg_fpa)
        pub_fpa.publish(msg_txt)
        pub_fpa.publish(msg_ecl)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
