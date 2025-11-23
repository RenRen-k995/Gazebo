import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

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
        
        self.get_logger().info('Obstacle Avoidance Node Started')

    def listener_callback(self, msg):
        # 1. READ THE SENSOR
        # Filter out invalid readings (inf and nan)
        ranges = [r if not (math.isinf(r) or math.isnan(r)) else msg.range_max 
                  for r in msg.ranges]
        
        if not ranges:
            self.get_logger().warn('No valid laser scan data')
            return
        
        # Get distances in different sectors
        # Front: center of the scan
        mid_index = len(ranges) // 2
        front_range = 30  # Check 30 degrees in front
        
        # Front sector
        front_start = max(0, mid_index - front_range)
        front_end = min(len(ranges), mid_index + front_range)
        front_distance = min(ranges[front_start:front_end])
        
        # Left sector (assuming counterclockwise scan)
        left_start = min(len(ranges) - 1, mid_index + 60)
        left_end = min(len(ranges), mid_index + 120)
        if left_end > left_start:
            left_distance = min(ranges[left_start:left_end])
        else:
            left_distance = msg.range_max
        
        # Right sector
        right_start = max(0, mid_index - 120)
        right_end = max(0, mid_index - 60)
        if right_end > right_start:
            right_distance = min(ranges[right_start:right_end])
        else:
            right_distance = msg.range_max
        
        # Create the message to send to the wheels
        cmd = Twist()

        # 2. THE LOGIC
        obstacle_threshold = 1.5  # Stop if obstacle within 1.5m
        
        if front_distance < obstacle_threshold:
            # OBSTACLE DETECTED! 🛑
            self.get_logger().info(f'Obstacle detected at {front_distance:.2f}m - Avoiding')
            # Stop forward motion
            cmd.linear.x = 0.0
            
            # Turn towards the side with more space
            if left_distance > right_distance:
                # Turn left
                cmd.angular.z = 0.5
                self.get_logger().info('Turning left')
            else:
                # Turn right
                cmd.angular.z = -0.5
                self.get_logger().info('Turning right')
        else:
            # PATH CLEAR! 🏎️
            # Drive forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            
        # 3. DRIVE
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
