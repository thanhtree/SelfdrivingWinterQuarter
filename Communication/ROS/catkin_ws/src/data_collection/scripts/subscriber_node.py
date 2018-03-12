#!/usr/bin/env python

## Subscribes to lane line detection topics

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Header

# def cb_lane_line_center_offset(data):
#     rospy.loginfo('caller_id: ' +  rospy.get_caller_id() 
#         + ' center_offset: %f', data.data)

def cb_lane_line_center_offset(data):
    rospy.loginfo(' center_offset: %f', data.data)

def cb_lane_line_left_radius_of_curvature(data):
    rospy.loginfo(' left_radius_of_curvature: %f', data.data)

def cb_lane_line_right_radius_of_curvature(data):
    rospy.loginfo(' right_radius_of_curvature: %f', data.data)

# def cb_rplidar_scan(data):
#     rospy.loginfo(' rplidar_scan: %f', data.data)

def subscriber_node():

    rospy.init_node('subscriber_node', anonymous=True)

    rospy.Subscriber('lane_line_center_offset', Float64, 
        cb_lane_line_center_offset)
    rospy.Subscriber('lane_line_left_radius_of_curvature', Float64, 
        cb_lane_line_left_radius_of_curvature)
    rospy.Subscriber('lane_line_right_radius_of_curvature', Float64, 
        cb_lane_line_right_radius_of_curvature)
    # rospy.Subscriber('scan', sensor_msgs/LaserScan, 
    #     cb_rplidar_scan)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber_node()
