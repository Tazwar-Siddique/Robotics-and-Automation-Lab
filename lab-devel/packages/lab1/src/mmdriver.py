#!/usr/bin/env python3
import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped
# Import the message for the wheel comm
class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher('/ee483mm11/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10) #publisher


    def drive(self): # CHANGE TO THE NAME OF YOUR FUNCTION

        # FIRST TURN
        # just need it to run 
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0
        self.pub.publish(cmd_to_publish)
        rospy.sleep(2)

        print("Driving the MM " + self.veh_name + " around the block") # Just for testin
#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.25
        cmd_to_publish.vel_left = 0.3
        self.pub.publish(cmd_to_publish)
        rospy.sleep(4)

        print("Stop before turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)
        
        print("Turn left")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.3
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(1)

        print("Stop after turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)

# SECOND TURN
        # just need it to run 
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0
        self.pub.publish(cmd_to_publish)
        rospy.sleep(2)

        print("Driving the MM " + self.veh_name + " around the block") # Just for testin
#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.25
        cmd_to_publish.vel_left = 0.3
        self.pub.publish(cmd_to_publish)
        rospy.sleep(4)

        print("Stop before turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)
        
        print("Turn left")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.3
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(1)

        print("Stop after turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)

# THIRD TURN
        # just need it to run 
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0
        self.pub.publish(cmd_to_publish)
        rospy.sleep(2)

        print("Driving the MM " + self.veh_name + " around the block") # Just for testin
#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.25
        cmd_to_publish.vel_left = 0.3
        self.pub.publish(cmd_to_publish)
        rospy.sleep(4)

        print("Stop before turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)
        
        print("Turn left")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.3
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(1)

        print("Stop after turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)

# FORTH TURN
        # just need it to run 
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0
        self.pub.publish(cmd_to_publish)
        rospy.sleep(2)

        print("Driving the MM " + self.veh_name + " around the block") # Just for testin
#WRITE THE CODE TO MAKE THE MM GO AROUND THE BLOCK
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.25
        cmd_to_publish.vel_left = 0.3
        self.pub.publish(cmd_to_publish)
        rospy.sleep(4)

        print("Stop before turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)
        
        print("Turn left")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0.3
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(1)

        print("Stop after turn")
        cmd_to_publish = WheelsCmdStamped()
        cmd_to_publish.header.stamp = rospy.Time.now()
        cmd_to_publish.vel_right = 0
        cmd_to_publish.vel_left = 0

        self.pub.publish(cmd_to_publish)
        rospy.sleep(5)


if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
         # Delay to wait enough time for the code to run
# Keep the line above - you might be able to reduce the delay a bit,
        #while not rospy.is_shutdown(): # Run ros forever - you can change
# this as well instead of running forever
        drive.drive()
    except rospy.ROSInterruptException:
        pass