#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, LanePose

class EMAFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None

    def update(self, new_measurement):
        if self.filtered_value is None:
            self.filtered_value = new_measurement
            return new_measurement
        
        # EMA Formula
        self.filtered_value = (self.alpha * new_measurement) + \
                              ((1.0 - self.alpha) * self.filtered_value)
        
        return self.filtered_value

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_int = 0
        self.prev_error = None

    def calc_p(self, error):
        return self.kp*error
    
    def calc_i(self, error, dt, res_err_int = False):
        if res_err_int:
            self.error_int = 0
        
        calc = self.ki * (error * dt + self.error_int)
        self.error_int += error * dt
        return calc
    
    def calc_d(self, error, dt):
        if self.prev_error == None:
            self.prev_error = error
            return 0
        else:
            d_err = error - self.prev_error
            self.prev_error = error
            return self.kd * d_err / dt if dt > 0 else 0
        

class Controller:
    def __init__(self):
        rospy.set_param("controller_ready", "true")
        
        rospy.Subscriber("/ee483mm11/lane_filter_node/lane_pose", LanePose, callback = self.error_callback)
        
        self.pub = rospy.Publisher("/ee483mm11/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=100)

        self.kp = rospy.get_param("~kp",3) 
        self.ki = rospy.get_param("~ki",0.1)
        self.kd = rospy.get_param("~kd",0.5)
        
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.25)
        self.angle_filter = EMAFilter(self.filter_alpha)
        
        self.pub_filtered_phi = rospy.Publisher(
            "/ee483mm11/lane_keeping_controller/filtered_phi", 
            Float32, 
            queue_size=10
        )

        self.prev_time = None
        self.pid_controller = PID(self.kp, self.ki, self.kd)
    
    def error_callback(self,msg):
        
        if self.prev_time == None:
            self.prev_time = rospy.Time.now()

        filtered_phi = self.angle_filter.update(msg.phi)
        
        car_cmd = Twist2DStamped()
        car_cmd.v = 0 
        
        error = -1 * filtered_phi 

        filtered_msg = Float32()
        filtered_msg.data = filtered_phi
        self.pub_filtered_phi.publish(filtered_msg)

        rospy.loginfo(f"Error value: {error}")
        
        dt = (rospy.Time.now() - self.prev_time).to_sec()
        self.prev_time = rospy.Time.now()
        
        # PID calculation uses the filtered error
        control_input = self.pid_controller.calc_p(error) + \
                        self.pid_controller.calc_i(error, dt) + \
                        self.pid_controller.calc_d(error, dt)
        
        car_cmd.omega = control_input

        rospy.loginfo(f"Publishing control: {car_cmd.omega}")
        self.pub.publish(car_cmd)
    
    def send_zero_commands(self):
        car_cmd = Twist2DStamped()
        car_cmd.v = 0
        car_cmd.omega = 0
        self.pub.publish(car_cmd)

if __name__=="__main__":
    rospy.init_node("controller", anonymous=True)
    rospy.loginfo("Controller Node Starting")
    try:
        ct = Controller()
        rospy.on_shutdown(ct.send_zero_commands)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
