import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Publisher: Send velocity commands to move the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber: Listen to the Lidar data
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.listener_callback, 
            10)

    def listener_callback(self, msg):
        # 1. READ THE SENSOR
        # The Lidar gives us a list of distances. 
        # With the plugin we used (-3.14 to 3.14), the "Front" is in the middle of the list.
        mid_index = len(msg.ranges) // 2
        front_distance = msg.ranges[mid_index]
        
        # Create the message to send to the wheels
        cmd = Twist()

        # 2. THE LOGIC
        if front_distance < 2.0:
            # OBSTACLE DETECTED! 🛑
            # Stop and turn left
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            # PATH CLEAR! 🏎️
            # Drive forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            
        # 3. DRIVE
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()