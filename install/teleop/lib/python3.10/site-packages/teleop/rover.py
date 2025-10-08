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
import logging

# Set up basic configuration for logging
logging.basicConfig(level=logging.DEBUG)




class MotorController(Node):

    def __init__(self):
        super().__init__('jimmy_motor_controller')
        self.serial_port = '/dev/ttyACM0'  # Adjust the port based on your Arduino connection
        self.serial_baudrate = 115200
        #self.serial_connection = serial.Serial(self.serial_port, self.serial_baudrate)
        self.arm_ready = 0  # Initialize as an instance variable
        self.griper = 1 #0=closed & 1=open
        self.control="rover"

        #sub to joy
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)#on vera pour mettre a 1 le buff jetrouve ça plus réactif
        
        #sub to cmd_vel
        self.subscription_twist = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            1)
        
        # Wheel geometry constants
        self.LFy = 0.27428
        self.RFy = 0.27428
        self.RBy = 0.133
        self.LBy = 0.133
        self.LFx = 0.184
        self.RFx = 0.184
        self.RBx = 0.133
        self.LBx = 0.133
        self.Mx = 0.254

    def twist_callback(self, msg):
        print("twist_call")
        command = self.convert_twist_to_command(msg)
        self.send_command_to_arduino(command)
    def get_target_radius(self, twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        direction = "R" if angular_velocity < 0 else "L"
        if angular_velocity != 0 :
        	target_radius = abs(linear_velocity / angular_velocity)*1000 
        else:
        	target_radius=0
        if target_radius > 0 and target_radius<500:
        	target_radius=500
        if target_radius > 0 and target_radius>2500:
            target_radius=2500
        return target_radius, direction

    def convert_twist_to_command(self, twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        if (angular_velocity !=0):
            target_radius,direction=self.get_target_radius(twist_msg)
            target_radius=int(target_radius)
            formatted_radius = f"{target_radius:04d}"
            output_command = "R" + formatted_radius + direction+'\r'
            self.send_command_to_arduino(output_command)
        if (linear_velocity >0):
            self.send_command_to_arduino("F"+'\r')
        elif (linear_velocity<0):
            self.send_command_to_arduino("B"+'\r')
        else:
            self.send_command_to_arduino("S"+'\r')

    def joy_callback(self, msg):
        #print("joy_call")
        command = self.convert_joy_to_command(msg)
        self.send_command_to_arduino(command)

    def convert_joy_to_command(self, joy_msg):
        global arm_ready
        forward = joy_msg.axes[1]  #forward
        up = joy_msg.axes[4]  #Up/down (right joystick)
        angle = joy_msg.axes[0]  #angle of the head
        
        
        btnA = joy_msg.buttons[0]  #A -> close
        btnB = joy_msg.buttons[1]  #B -> Open
        btnX = joy_msg.buttons[2]  #X -> fold
        btnY = joy_msg.buttons[3]  #Y -> Unfold
        JoyL = joy_msg.buttons[9] #stop
        JoyR = joy_msg.buttons[10] #stop
        RB = joy_msg.buttons[5] # change to rover 
        LR = joy_msg.buttons[4] # change to jimmy
        #btnBoot = joy_msg.buttons[11] # reboot
        #TODO was commented, check why it was buggy

        if (RB == 1):
                    self.control = "rover"
                    return "Joff" +'\r'
        if (LR == 1):
                    self.control = "jimmy"
                    return "Jon"+'\r'
        if(self.control=="jimmy"):
            print("we're waiting for jimmy cmd")
            if(joy_msg.axes[2]<-0.5 and joy_msg.axes[5]<-0.5): #add this line with good incrementation for kill switch 
                if (JoyL == 1 or JoyR ==1):
                    return "JS"+'\r'
                #elif btnBoot == 1:
                #    return "Jreboot"+'\r'
                elif btnA == 1:
                    return "JCl"+'\r'
                elif btnB == 1:
                    return "JOp"+'\r'
                elif btnX == 1:
                    return "JBa"+'\r'
                elif btnY == 1:
                    return "JGo"+'\r'
                elif forward>0.3:
                    return "JF"+'\r'
                elif forward<-0.3:
                    return "JB"+'\r'
                elif up>0.3:
                    return "JUp"+'\r'
                elif up<-0.3:
                    return "JDo"+'\r'#for down
                elif angle>0.3:
                    return "Jle"+'\r'#for left
                elif angle<-0.3:
                    return "JRi"+'\r'#for right
                else:
                    return 
        
        
        
    def send_command_to_arduino(self, command):
        if command!=None:
            print(f"message sent to Arduino: {command}")
            previousTime=time.time()
            #self.serial_connection.write(command.encode())
            #self.serial_connection.flush()

            """
            while True:
                line = self.serial_connection.readline().decode().strip()
                if line != "":
                    currentTime=time.time()
                    #print(line)
                    #print('time difference was: ', str(currentTime-previousTime))
                    break
            """



def main(args=None):
    print("Starting Motor Controller Node...")
    rclpy.init(args=args)  # Ensure to pass args to rclpy.init
    
    motor_controller = MotorController()  # Assuming this is the correct class to instantiate

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(motor_controller)

    try:
        executor.spin()  # Directly call spin without threading
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()  # Properly shutdown the executor
        rclpy.shutdown()

if __name__ == '__main__':
    main()