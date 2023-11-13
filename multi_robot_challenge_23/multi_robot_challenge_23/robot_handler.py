import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose


class RobotHandlerClass(Node):
    def __init__(self):
        super().__init__('RobotHandlerNode')

        self.sub_lidar = self.create_subscription(LaserScan, 'scan', self.clbk_lidar, 10)
        
        self.pub_namespace_test = self.create_publisher(Float64, 'namespace_test', 10)
        self.marker_id = self.create_subscription(Int64, 'marker_id', self.clbk_marker_id,10)
        self.marker_id_map_pose = self.create_subscription(Pose,'marker_map_pose', self.clbk_map_pose,10)
        self.lidar_value = 100.0

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_lidar(self, msg):
        self.lidar_value = msg.ranges[180]
    
    def clbk_marker_id(self, msg):
        self.marker_id = msg.data
        self.get_logger().info(f"{msg.data}")
       
    
    def clbk_map_pose(self, msg):
        print("called on map_pose")
        self.marker_id_map_pose = msg.position
        print()
        self.get_logger().info(f"X: {msg.position.x} Y. {msg.position.y}")

    def timer_callback(self):
        pub_msg = Float64()
        pub_msg.data = self.lidar_value
        self.pub_namespace_test.publish(pub_msg)
       # print(self.marker_id_map_pose)
        


def main(args=None):
    rclpy.init(args=args)

    robot_handler = RobotHandlerClass()

    rclpy.spin(robot_handler)

    robot_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()