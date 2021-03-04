#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose, FSMState
import numpy as np
import time

class ControllerNode(DTROS):

    def __init__(self,node_name):
        # Initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        #Publisher
        self.pub_car_cmd = rospy.Publisher("marschla/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        #Subscriber
        self.sub_lane_reading = rospy.Subscriber("marschla/lane_filter_node/lane_pose", LanePose, self.control, "lane_filter", queue_size=1,buff_size=(20*(1024**2)))
        #self.sub_fsm_state = rospy.Subscriber("marcobot/fsm_node/mode", FSMState, self.state, queue_size=1)    
    
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)

        self.done = False

        #sys params
        self.vref = 0.23    #v_ref defines speed at which the robot moves 

        #params used for PID control 
        self.C_p = 0.0
        self.C_i = 0.0

        self.L = 0.05

        self.tnew = time.time()

    def resetintegral(self,C_i,dist,phi):
        tol_d = 0.05
        tol_phi = 0.15

        if np.absolute(dist) <= tol_d and np.absolute(phi) <= tol_phi:
            rospy.loginfo("Reset Integral")
            return 0.0
        else:
            return C_i


    def getomega(self,dist,tist,dt):
        #parameters for PID control
        k_p = 4.0
        k_i = 0.0
        #saturation params
        sati = 1.0
        omegasat=4.0
        

        err = 6*dist+1.5*tist

        #proportional gain part
        self.C_p = k_p*err

        #integral term (approximate integral)
        self.C_i += k_i*dt*err

        
        #make sure integral term doesnt become too big
        if self.C_i > sati:
            self.C_i = sati
        if self.C_i < -sati:
            self.C_i = -sati

        self.C_i = self.resetintegral(self.C_i,dist,tist)
        
        #computing control output
        omega = self.C_p + self.C_i 
        
        
        if omega>omegasat:
            omega=omegasat
        if omega<-omegasat:
            omega=-omegasat
        
        return omega


    #shutdown procedure, stopping motor movement etc.
    def custom_shutdown(self):

        self.done = True

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

        #rospy.loginfo("Hallo")

        told = self.tnew
        self.tnew = time.time()
        dt = self.tnew - told

        rospy.loginfo("dt = %s" % dt)

        delay = rospy.Time.now() - pose.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay [s] =  %s' % delay_float)  

        omega = self.getomega(pose.d,pose.phi,dt)

        car_cmd_msg = WheelsCmdStamped()
        
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.vel_left = self.vref + self.L * omega
        car_cmd_msg.vel_right = self.vref - self.L * omega

        if self.done == False:
            self.pub_car_cmd.publish(car_cmd_msg)
            rospy.loginfo("Hallo +++")


if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    #lane_controller_node.run()
    # Keep it spinning
    rospy.spin()