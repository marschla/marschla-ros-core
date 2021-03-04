#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose, FSMState, SegmentList
import numpy as np
import time
from sensor_msgs.msg import CompressedImage

class ControllerNode(DTROS):

    def __init__(self,node_name):
        # Initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        #Publisher
        self.pub_car_cmd = rospy.Publisher("marschla/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        #Subscriber
        self.sub_lane_reading = rospy.Subscriber("marschla/lane_filter_node/lane_pose", LanePose, self.control, "lane_filter", queue_size=1)
        #self.sub_fsm_state = rospy.Subscriber("marcobot/fsm_node/mode", FSMState, self.state, queue_size=1)   
        self.sub_seg = rospy.Subscriber("marschla/ground_projection_node/lineseglist_out", SegmentList, self.process_segments, queue_size=1)
        self.sub_lineseglist_ = rospy.Subscriber("marschla/line_detector_node/segment_list", SegmentList, self.lineseglist_cb, queue_size=1)
        #self.sub_image = rospy.Subscriber("marschla/camera_node/image/compressed", CompressedImage, self.image_cb, buff_size=10000000, queue_size=1)    

        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)



    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        
        while  not rospy.is_shutdown():


            rate.sleep()

    #shutdown procedure, stopping motor movement etc.
    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_car_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")

    #function updates pose variables, that camera gives us data at higher rate then this code operates at,
    #thus we do not use all incoming data
    def control(self,pose, source):
        '''
        self.dist = pose.d
        self.tist = pose.phi
        #self.header = pose.header
        message = pose.d
        #rospy.loginfo('d =  %s' % message)
        '''
        delay = rospy.Time.now() - pose.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay lanepose [s] =  %s' % delay_float)  

    def process_segments(self,msg):
        delay = rospy.Time.now() - msg.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay seglistout [s] =  %s' % delay_float)  

    def lineseglist_cb(self,msg):
        delay = rospy.Time.now() - msg.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay seglistin [s] =  %s' % delay_float)  

    def image_cb(self,msg):
        delay = rospy.Time.now() - msg.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay img [s] =  %s' % delay_float)  




if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    #lane_controller_node.run()
    # Keep it spinning
    rospy.spin()