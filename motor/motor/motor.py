#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile
import odrive
from odrive.enums import *
import math
import numpy as np
import serial
import os

class Motor(Node):
    SERIAL_PORT = '/dev/ttyACM0'
    odrv0 = odrive.find_any()
    def __init__(self):
        super().__init__("motor")
        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found")
            rclpy.shutdown()
        self.ser = serial.Serial(self.SERIAL_PORT,115200)
        #self.send_serial("")
        self.twist_subscriber = self.create_subscription(Twist,"/cmd_vel",self.send_cmd_vel,10)
        self.get_logger().info("motor has started")
        self.position_publisher=self.create_publisher(Float32MultiArray, "/position",10)
        self.timer_=self.create_timer(0.02,self.pub_velocity)
        self.count=0.0


    def send_cmd_vel(self,msg):
        self.get_logger().info("Twist: Linear : %f Angular velocity: %f" % (msg.linear.x, msg.angular.z))
        self.odrv0.axis1.controller.input_vel=(msg.linear.x+msg.angular.z*0.54/2)/0.1
        self.odrv0.axis0.controller.input_vel=(msg.linear.x-msg.angular.z*0.54/2)/0.1

    def pub_velocity(self):
        msg = Float32MultiArray()
        wheel_r_pos=self.odrv0.axis0.encoder.pos_estimate*2*math.pi/17
        wheel_l_pos=self.odrv0.axis1.encoder.pos_estimate*2*math.pi/17
        wheel_r_vel=self.odrv0.axis0.encoder.vel_estimate*0.1
        wheel_l_vel=self.odrv0.axis1.encoder.vel_estimate*0.1
        count=self.count
        msg.data =[wheel_r_pos,wheel_l_pos,wheel_r_vel,wheel_l_vel,count]
        self.position_publisher.publish(msg)
        self.count += 1.0
        


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    
    main()