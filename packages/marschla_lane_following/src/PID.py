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
        #self.pub_car_cmd = rospy.Publisher("marschla/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        #self.pub_car_cmd = rospy.Publisher("marschla/lane_controller_node/car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        self.pub_car_cmd = rospy.Publisher("marschla/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1,dt_topic_type=TopicType.CONTROL)

        #Subscriber
        self.sub_lane_reading = rospy.Subscriber("marschla/lane_filter_node/lane_pose", LanePose, self.control, "lane_filter", queue_size=1,buff_size=(20*(1024**2)))
        #self.sub_fsm_state = rospy.Subscriber("marcobot/fsm_node/mode", FSMState, self.state, queue_size=1)    
    
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)

        #sys params
        self.vdiff = 0.0
        self.omega = 0.0
        self.vref = 0.23    #v_ref defines speed at which the robot moves 
        self.dist = 0.0
        self.dold = 0.0
        self.tist = 0.0

        #params used for PID control 
        self.C_p = 0.0
        self.C_i = 0.0
        self.C_d = 0.0

        self.L = 0.05

        self.header = 0

        self.state_var = "none"

    def getomega(self,dist,tist,dt):
        #parameters for PID control
        k_p = 4.0
        k_i = 0.0
        k_d = 0.0
        #saturation params
        sati = 1.0
        satd = 1.0
        omegasat=4.5
        

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
        
        #derivative term
        self.C_d = k_d*(err-self.dold)/dt
        self.dold = err
        
        #make sure derivative term doesnt become too big
        if self.C_d > satd:
            self.C_d = satd
        if self.C_d < -satd:
            self.C_d = -satd
        
        #computing control output
        omega = self.C_p + self.C_i + self.C_d

        #rospy.loginfo("C_i = %s" % self.C_i)
        
        
        if omega>omegasat:
            omega=omegasat
        if omega<-omegasat:
            omega=-omegasat
        
        return omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        stoptime = 28.0
        t0 = time.time()
        i=0
        
        #wait for data to be published
        wait = rospy.Rate(0.05)
        wait.sleep()

        rospy.logwarn("Hallo --------------------")
        
        while  not rospy.is_shutdown():


            
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told

            #self.vdiff = self.getvdiff(self.dist,self.tist,dt)
            self.omega = self.getomega(self.dist,self.tist,dt)

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref + self.L * self.omega
            car_cmd_msg.vel_right = self.vref - self.L * self.omega
            #car_cmd_msg.v = self.vref
            #car_cmd_msg.omega = -self.omega

            self.pub_car_cmd.publish(car_cmd_msg)

            #rospy.loginfo("HAllo")

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lane_pose
            message1 = self.dist
            message2 = self.omega
            message3 = self.tist
            message4 = dt

            
            rate.sleep()

    #shutdown procedure, stopping motor movement etc.
    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_car_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")

    #function updates pose variables, that camera gives us data at higher rate then this code operates at,
    #thus we do not use all incoming data
    def control(self,pose, source):
        self.dist = pose.d
        self.tist = pose.phi
        #self.header = pose.header
        message = pose.d
        #rospy.loginfo('d =  %s' % message)
        delay = rospy.Time.now() - pose.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay [s] =  %s' % delay_float)  


if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()