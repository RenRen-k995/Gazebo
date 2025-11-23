import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        # Initialize the broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the Odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom_pose,
            10)
        
        self.get_logger().info('TF Broadcaster Node Started - Publishing odom->base_link transform')

    def handle_odom_pose(self, msg):
        try:
            t = TransformStamped()

            # Read the time and frames from the Odom message
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            # Copy the position (Translation)
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z

            # Copy the orientation (Rotation)
            t.transform.rotation = msg.pose.pose.orientation

            # Send the transform
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
