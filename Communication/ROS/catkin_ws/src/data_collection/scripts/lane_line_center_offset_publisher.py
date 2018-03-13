#!/usr/bin/env python
#
#  Publishes lane line center offset data to 'lane_line_center_offset' topic
#

import rospy
from std_msgs.msg import Float64
import zmq
import time

def lane_line_center_offset_publisher():

    context = zmq.Context()

    # connect subscriber socket
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.IDENTITY, "Hello")
    subscriber.setsockopt(zmq.SUBSCRIBE, "")
    subscriber.connect("tcp://localhost:5565")

    # synchronize with the publisher
    sync = context.socket(zmq.PUSH)
    sync.connect("tcp://localhost:5564")
    sync.send("")  

    pub_lane_line_center_offset = rospy.Publisher('lane_line_center_offset', 
        Float64, queue_size=10)

    rospy.init_node('lane_line_center_offset_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # obtain values from lane line detection process
        center_offset = subscriber.recv() 
        print(data)

        rospy.loginfo("center_offset: %f" % center_offset)

        pub_lane_line_center_offset.publish(center_offset)

        rate.sleep()

if __name__ == '__main__':
    try:
        lane_line_center_offset_publisher()
    except rospy.ROSInterruptException:
        pass
