# 📑 System Overview

Adversity is an academic project and a demonstrator of mobile robotics. ROS2 is used to control the rover. The rover was engineered to be used indoors. 

The system is composed of:
- Actuators (6 DC Motors for wheels, 4 servo motors for the steering mechanism, 4 Dynamixels servo motors for the robotic arm) 
- A sensor (a Lidar than was used for SLAM, simultaneous localization and mapping)
- 2 Electronic boards (one for power management and one for the control of actuators)
- A Raspberry Pi that communicates with the control board and the Lidar

# 🧠 Communication and High-Level Control

ROS2 allows for complex application such as SLAM and autonomous navigation. This is why the problem is decomposed into layers in the hardware and communication. This approach is similar to how a CNC machine works (Numerical Commands deal with complex computations and inputs and outputs management is done by a PLC).

For **autonomous navigation and SLAM** use case:

1. The Lidar returns data to the Raspberry Pi
2. The Raspberry Pi publishes the data on a network
3. A PC with ROS2 is connected to the network constructs the map and gets user inputs to produce cartesian velocity commands for the vehicle
4. The Raspberry Pi gets this information, transforms the commands to a turning radius and   sends a message to the control board
5. The control boards converts the turning radius into motor commands following Ackerman Steering Geometry

For **tele-operation**, no external ROS2 PC is needed:

1. An external PC connected to the Network opens a Raspberry Pi terminal via SSH and launches a ROS2 package listening to keyboard inputs
2. The Raspberry Pi gets this keyboard information, transforms the commands to a turning radius and  and sends a message to the control board
3. The control boards converts the turning radius into motor commands following Ackerman Steering Geometry

From this operation mode, it becomes clear that having a fix IP address for the Raspberry Pi is convenient.  A small router was used so that IP address are controlled and that information broadcasting in ROS2 is not done on a large public network.

# 🤓 ROS2 Notions
**Robot Operating System (ROS)** is a set of software libraries and tools for building robot applications. In the project, communications and messages from ROS2 are used to make processes interact. 

Please have a look at the beginning of ROS2 documentation. Also not that ROS2 is intended to run on linux (for the compiler among other reasons).  Here are nonetheless some important concept from ROS2 for the rover:

## 🏭 Workspace
A workspace is a directory containing ROS2 packages.

## 📦 Packages
A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

##  🧠 Nodes

Each Node in ROS is responsible for a single modular purpose (listening to keyboard inputs and making velocity commands for example). Each node can communicate via:
- topics (*publisher/subscriber* manner, used on the rover)
- services
- actions

## 🚀  Launch

To launch a node, you can run directly the node from the package it sits in or you can launch a "launch file" that you wrote and that file can launch multiple nodes. It is convenient as it reduces the number of commands to start all nodes needed. 

| Command | Meaning |
| :--- | :--- |
| `ros2 run my_package my_node` | run a node |
| `ros2 launch my_package launch.py` | launch a launch file |

Launch file are usually written in python and looks like that:

```python 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

With a bit of ROS2 knowledge, launch file are easily understood. 

# 🔬 Control Board 

## 🏎️ Rover Control

The control boards (whose PCB documentation is available) receives an encoded velocity request from the Raspberry Pi. The code running on the Arduino decodes it  and converts the request to motor commands ensuring respect of the Ackermann steering geometry (c.f report). The code used is called `full_control.ino`

The code ensures that motor commands are sent with respect to the request coming from the Raspberry Pi and the current state of the rover. To do so, some variables are the key:

```C
long unsigned targetNav2; // keeps track of current turning radius
char isMooving='n'; // keeps track of wether the rover is mooving (backward or forward) or not
```

For example, if the Arduino/control board receive an instruction to turn move forward, `targetNav2` is used to know if speed differential must be applied as the rover might be in a curve already:

```c
  if (command=="F"){
      digitalWrite(blueLed,HIGH);
      if (targetNav2==0){
        Adversity.keepRoverAtSpeed("forward", 255);
      }
      else {
        Adversity.speedSteer("forward",steerDirection,targetNav2);
      }
      isMooving='f';
      while (Serial.available()<0){};
      digitalWrite(blueLed,LOW);
  }
```

Reading through the code, in the function `Executor` will allow you to understand the decision tree. 

The classes `DCMotor`, `Rover_Calculations`, `Rover` contain all methods needed for controlling the rover.

## 🦾  Robotic Arm Control

The Robot Arm attached to the rover is called `jimmy` in the code. This robot is made of XL320 Dynamixels motors, a library is used to control them. Jimmy has a small workspace and is only controlled in joint mode.

# 💡 Tips

- DC motor differential with Ackermann Steering doesn't work when moving backwards, because of a PWM conflict
- If a DC motor turns in the wrong direction, you can swap its wires 
- It seems that `batteryWarning()` doesn't properly detects that the battery is low even if test bench worked, more investigation is needed 

# 📋 Summary for Tele-Operation

Use a terminal from your PC to access the raspberry Pi via ssh, example:

```bash
ssh rover@192.168.8.111
```

Upload arduino code from the Raspberry Pi to the Arduino board

```bash
cd ~/PT_Rover/Arduino/full_control
arduino --upload full_control.ino --port /dev/ttyACM0`
```

Prerequisites are:
- Arduino is installed on the Raspberry Pi
- Check if the port is `ttyACM0` or another
- Give the read/write rights to the port 

Then launch ros2 nodes and teleoperate the rover:

- **i forward**
-  **m backward**
- **k stop**
- **j stearLeft**
- **l stearRight**
