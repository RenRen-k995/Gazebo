import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):
    # Sector angle offsets for obstacle detection (in scan array indices)
    FRONT_SECTOR_RANGE = 30  # Check 30 degrees in front
    LEFT_SECTOR_START = 60
    LEFT_SECTOR_END = 120
    RIGHT_SECTOR_START = 120
    RIGHT_SECTOR_END = 60
    
    # Obstacle detection threshold in meters
    OBSTACLE_THRESHOLD = 1.5
    
    # Movement parameters
    FORWARD_SPEED = 0.3
    TURN_SPEED = 0.5
    
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
    
    def filter_laser_ranges(self, msg):
        """Filter out invalid laser scan readings (inf and nan)"""
        return [r if not (math.isinf(r) or math.isnan(r)) else msg.range_max 
                for r in msg.ranges]

    def listener_callback(self, msg):
        # 1. READ THE SENSOR
        # Filter out invalid readings (inf and nan)
        ranges = self.filter_laser_ranges(msg)
        
        if not ranges:
            self.get_logger().warn('No valid laser scan data')
            return
        
        # Get distances in different sectors
        # Front: center of the scan
        mid_index = len(ranges) // 2
        
        # Front sector
        front_start = max(0, mid_index - self.FRONT_SECTOR_RANGE)
        front_end = min(len(ranges), mid_index + self.FRONT_SECTOR_RANGE)
        front_distance = min(ranges[front_start:front_end])
        
        # Left sector (assuming counterclockwise scan)
        left_start = min(len(ranges) - 1, mid_index + self.LEFT_SECTOR_START)
        left_end = min(len(ranges), mid_index + self.LEFT_SECTOR_END)
        if left_end > left_start:
            left_distance = min(ranges[left_start:left_end])
        else:
            left_distance = msg.range_max
        
        # Right sector
        right_start = max(0, mid_index - self.RIGHT_SECTOR_START)
        right_end = max(0, mid_index - self.RIGHT_SECTOR_END)
        if right_end > right_start:
            right_distance = min(ranges[right_start:right_end])
        else:
            right_distance = msg.range_max
        
        # Create the message to send to the wheels
        cmd = Twist()

        # 2. THE LOGIC
        if front_distance < self.OBSTACLE_THRESHOLD:
            # OBSTACLE DETECTED! 🛑
            self.get_logger().info(f'Obstacle detected at {front_distance:.2f}m - Avoiding')
            # Stop forward motion
            cmd.linear.x = 0.0
            
            # Turn towards the side with more space
            if left_distance > right_distance:
                # Turn left
                cmd.angular.z = self.TURN_SPEED
                self.get_logger().info('Turning left')
            else:
                # Turn right
                cmd.angular.z = -self.TURN_SPEED
                self.get_logger().info('Turning right')
        else:
            # PATH CLEAR! 🏎️
            # Drive forward
            cmd.linear.x = self.FORWARD_SPEED
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
