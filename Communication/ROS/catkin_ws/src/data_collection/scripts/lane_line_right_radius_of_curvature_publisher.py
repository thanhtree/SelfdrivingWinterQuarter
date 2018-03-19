#!/usr/bin/env python

## Publishes lane line radius of curvature data to the
## 'lane_line_right_radius_of_curvature' topic

import rospy
import random
from std_msgs.msg import Float64

def lane_line_right_radius_of_curvature_publisher():
    # We cannot publish multiple data types to a single topic. 
    # Therefore we need to publish multiple topics from a single node
    pub = rospy.Publisher('lane_line_right_radius_of_curvature', 
        Float64, queue_size=10)

    rospy.init_node('lane_line_right_radius_of_curvature_publisher', 
        anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        right_radius_of_curvature = random.random() * 180

        rospy.loginfo("right_radius_of_curvature: %f" 
            % right_radius_of_curvature)

        pub.publish(right_radius_of_curvature)

        rate.sleep()

if __name__ == '__main__':
    try:
        lane_line_right_radius_of_curvature_publisher()
    except rospy.ROSInterruptException:
        pass
