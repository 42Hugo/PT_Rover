#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial
import time


vel_msg = Twist()  

class Keyboard_subscriber(Node):

    def __init__(self):
        super().__init__('Keyboard_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg
        vel_msg=data

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.serial_port = '/dev/ttyACM0'  # Adjust the port based on your Arduino connection
        self.serial_baudrate = 115200
        self.serial_connection = serial.Serial(self.serial_port, self.serial_baudrate)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)

    def twist_callback(self, msg):
        command = self.convert_twist_to_command(msg)
        self.send_command_to_arduino(command)

    def convert_twist_to_command(self, twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        
        if linear_velocity > 0:
            return "F"+'\r'
            #return  str(angular_velocity) # Forward motion
        elif linear_velocity < 0:
            return "B"+'\r'
            #return str(angular_velocity)  # Backward motion
        elif linear_velocity==0 and angular_velocity !=0:
            if (angular_velocity <0):
                print(angular_velocity)
                return "R"+'\r'
            else:
                print(angular_velocity)
                return "L"+'\r'
        else:
            return "S"+'\r'
            #return str(angular_velocity)  # Stop (no motion)
        
    def send_command_to_arduino(self, command):
        print(command)
        previousTime=time.time()
        self.serial_connection.write(command.encode())
        self.serial_connection.flush()
        
        while True:
            line = self.serial_connection.readline().decode().strip()
            if line != "":
                currentTime=time.time()
                print(line)
                print('time difference was: ', str(currentTime-previousTime))
                break
              


def main(args=None):
    rclpy.init(args=None)
    
    Keyboard_subscriber_inst = Keyboard_subscriber()
    motor_controller = MotorController()  

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(Keyboard_subscriber_inst)
    executor.add_node(motor_controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()    
    rate = motor_controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
