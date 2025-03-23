#!/usr/bin/env python

import sys
import os
import rospy
import serial
from std_msgs.msg import Bool, Float64

# Get script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
from lewansoul_servo_bus import ServoBus

# Define constants
PORT_NECK = '/dev/ttyUSB0'
PORT_HEAD = '/dev/ttyUSB1'
BAUDRATE = 115200

horizontalMinLeft = 60
horizontalMaxRight = 160
horizontalMid = 105

verticalMin = 150
verticalMax = 250
verticalMid = 180

class HeadController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('head_controller', anonymous=True)
        
        # Initialize servo bus
        self.servo_bus = ServoBus(PORT_NECK, baudrate=BAUDRATE)
        self.ser_horizontal = self.servo_bus.get_servo(2)
        self.ser_vertical = self.servo_bus.get_servo(1)
        
        # Initialize serial connection for head
        self.head_bus = serial.Serial(PORT_HEAD, baudrate=BAUDRATE, timeout=1)
        
        # Buffer for communication
        self.buf = [0] * 10
        
        # Subscriber for topics
        rospy.Subscriber('/set_eye_lids_blink', Bool, self.eyes_callback, queue_size=1)
        rospy.Subscriber('/mouth_close_and_open', Bool, self.mouth_callback, queue_size=1)
        rospy.Subscriber('/horizontal_set_angle_time', Float64, self.neck_callback, queue_size=1)
        
    def mouth_callback(self, msg):
        if msg.data:  # If True is received
            self.mouthCloseAndOpen()
    
    def eyes_callback(self, msg):
        if msg.data:  # If True is received
            self.setEyelidsBlink()
            
    def neck_callback(self, msg):
        self.horizontalSetAngleTime(msg.data)
    
    def setEyelidsBlink(self):
        self.buf[0] = 255
        self.buf[1] = 13
        self.buf[2] = 255
        
        self.head_bus.write(bytearray(self.buf[:3]))
        rospy.loginfo("Mouth CLOSE AND OPEN")
        
        if self.head_bus.inWaiting() > 0:
            receive_data = self.head_bus.read(self.head_bus.inWaiting())
            rospy.loginfo(receive_data.decode('utf-8'))
        else:
            rospy.loginfo("No received data")
               
    def mouthCloseAndOpen(self):
        self.buf[0] = 255
        self.buf[1] = 33
        self.buf[2] = 255
        
        self.head_bus.write(bytearray(self.buf[:3]))
        rospy.loginfo("Mouth CLOSE AND OPEN")
        
        if self.head_bus.inWaiting() > 0:
            receive_data = self.head_bus.read(self.head_bus.inWaiting())
            rospy.loginfo(receive_data.decode('utf-8'))
        else:
            rospy.loginfo("No received data")
            
    def horizontalSetAngleTime(self,angle):
        if angle < horizontalMinLeft:
            angle = horizontalMinLeft
		
        elif angle > horizontalMaxRight:
            angle = horizontalMaxRight

        self.ser_horizontal.move_time_write(angle,1.5)
        rospy.loginfo(f"NECK horizontal angle: {angle}, time: 1.5")
            
    def run(self):
        # Keep node running
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = HeadController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error: %s", str(e))