import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

def twist_callback(msg):
    print("Received Twist: ", msg)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('twist_subscriber')
    qos_profile = QoSProfile(depth=10)  # Example QoS profile, adjust as needed
    subscriber = node.create_subscription(Twist, '/cmd_vel', twist_callback, qos_profile)
    subscriber  # prevent unused variable warning
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
