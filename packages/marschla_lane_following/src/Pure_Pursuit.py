#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose, SegmentList
import numpy as np
import time
import math

class ControllerNode(DTROS):
    def __init__(self,node_name):

        super(ControllerNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        self.v = 0
        self.vref = 0.20
        self.L = 0.05
        self.omega = 0.0
        self.tnew = 0.0

        self.dist = 0.0
        self.phi = 0.0
        self.in_lane = False

        self.lastomega = 0

        # Start rospy for this node
        #rospy.init_node("lane_controller_node", anonymous=False)

        # Subscriptions
        self.sub_seg = rospy.Subscriber("marschla/ground_projection_node/lineseglist_out", SegmentList, self.process_segments, queue_size=1,buff_size=(20*(1024**2)))
        
        #self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/lane_filter_node/lane_pose", LanePose ,self.updatepose ,queue_size = 1)
        # Publication
        self.pub_wheels_cmd = rospy.Publisher("marschla/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1,dt_topic_type=TopicType.CONTROL)

        # Stop on shutdown
        rospy.on_shutdown(self.custom_shutdown)



    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_wheels_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")



    def process_segments(self, input_segment_list):
        all_segments = input_segment_list.segments # this is a list of type Segment

        rospy.loginfo("Hallo from sub")

        lookahead = 0.25
        tol = 0.1

        num_yellow = 0
        num_white = 0

        num_yellow_total = 0
        num_white_total = 0

        yellow_arr_total = np.zeros(2)
        white_arr_total = np.zeros(2)

        yellow_arr = np.zeros(2)
        white_arr = np.zeros(2)

        #rospy.loginfo("Hallo1")

        flag = True

        for segment in all_segments:
            point0 = segment.points[0]
            point1 = segment.points[1]

            ave_point_x = (point0.x + point1.x)/2.0
            ave_point_y = (point0.y + point1.y)/2.0

            d = np.sqrt(ave_point_x**2 + ave_point_y**2)

            if segment.color == 1:    #yellow color 

                num_yellow_total += 1
                yellow_arr_total += np.array([ave_point_x,ave_point_y])

                if d < lookahead + tol and d > lookahead:
                    num_yellow += 1
                    yellow_arr += np.array([ave_point_x,ave_point_y])

            if segment.color == 0 and ave_point_y > -0.15 and ave_point_y < 0.15:     #white color 

                num_white_total += 1
                white_arr_total += np.array([ave_point_x,ave_point_y])

                if d < lookahead + tol and d > lookahead:
                    num_white += 1
                    white_arr += np.array([ave_point_x,ave_point_y])

        if num_white != 0 and num_yellow != 0:
            rospy.logwarn("both colors detected")

            ave_white = white_arr * 1. / num_white
            ave_yellow = yellow_arr * 1. / num_yellow

            ave_point = (ave_white + ave_yellow)/2.0

            if ave_point[1] > 0.05:
                ave_point[1] += 1.5*ave_point[1]
            if ave_point[1] < -0.05:
                ave_point[1] += 2.25*ave_point[1]

            #ave_point[1] += 0.05

            self.v = self.vref

        if num_white == 0 and num_yellow == 0:
            #no white/yellow segments detected 
            #figure a procedure, if camera doesn't pick up any segments in target range
            self.v = 0.0
            self.omega = -2.5

            if self.lastomega > 0:
                self.omega = 2.5
            elif self.lastomega < 0:
                self.omega = -2.5
            else:
                self.omega = 0.0

            rospy.loginfo("no color detected")

            '''
            if white_arr_total[1] < 0 and yellow_arr_total[1] > 0:
                self.omega = 0.0
                self.vref = 0.1
            elif white_arr_total[1] > 0 and yellow_arr_total[1] < 0:
                self.omega = -2.0
                self.vref = 0.1
            elif white_arr_total[1] > 0 and yellow_arr_total[1] == 0:
                self.omega = 2.0
                self.vref = 0.1
            elif num_yellow_total !=0 and num_white_total == 0:
                self.omega = -1.5
                self.vref = 0.1
            elif num_white_total !=0 and num_yellow_total == 0:
                self.omega = 1.5
                self.vref = 0.1
            else:
                self.omega = 1.0
                self.vref = 0.0
            '''


            flag = False

        if num_white == 0 and num_yellow != 0:
            #only yellow segments (probably too far to the left)
            rospy.loginfo("only yellow detected")

            ave_yellow = yellow_arr * 1. / num_yellow

            offset = -0.25
            ave_point = ave_yellow + np.array([0.0,offset])


            self.v = self.vref*0.85

        if num_yellow == 0 and num_white != 0:
            #only white segments (probably too far to the right of the lane)
            rospy.loginfo("only white detected")

            ave_white = white_arr * 1. / num_white 

            offset = 0.2

            if self.omega > 3.0:
                offset += 0.1
                
            ave_point = ave_white + np.array([0.0,offset])

            self.v = self.vref*0.85


        #rospy.loginfo("Flag: %s" % flag)
        
        rospy.loginfo("yellow: %s" % num_yellow)
        rospy.loginfo("white %s" % num_white)

        if flag == True:
            alpha = np.arctan2(ave_point[1],ave_point[0])

            self.omega = 5.5*self.v* np.sin(alpha)/lookahead

            self.lastomega = self.omega

            rospy.loginfo("target: %s" % ave_point)
           

    
    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            #rospy.loginfo("Hallo\n")

            car_cmd_msg = WheelsCmdStamped()

            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.v - self.L * self.omega
            car_cmd_msg.vel_right = self.v + self.L * self.omega

            
            msg1 = self.omega
            msg2 = self.vref

            #rospy.loginfo("omega: %s" % msg1)
            #rospy.loginfo("vref: %s" % msg2)
            

            # Send the command to the car
            self.pub_wheels_cmd.publish(car_cmd_msg)
            #rospy.loginfo("omega = %s" % self.omega)
            
            rate.sleep()

if __name__ == '__main__':
    # Initialize the node
    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()
