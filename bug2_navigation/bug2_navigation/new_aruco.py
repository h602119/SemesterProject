import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition
from geometry_msgs.msg import Pose 
from std_msgs.msg import Int64
from std_msgs.msg import Int8

from geometry_msgs.msg import Point 

class new_aruco(Node):
    def __init__(self):
        super().__init__("aruco_check")

        self.get_logger().info("new aruco Node created")

        self.scoring_client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.scoring_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #self.create_subscription(Int64, '/tb3_0/marker_id', self.marker_callback, 10)
        #self.create_subscription(Pose, '/tb3_0/aruco_poses', self.marker_poses, 10)
        #self.create_subscription(Int64, '/tb3_1/marker_id', self.marker_callback, 10)
        #self.create_subscription(Pose, '/tb3_0/aruco_poses', self.marker_poses, 10)


        self.create_subscription(ArucoMarkers, '/tb3_0/aruco_markers', self.marker_callback, 10)
        self.create_subscription(ArucoMarkers, '/tb3_1/aruco_markers', self.marker_callback, 10)
        

     
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        #Scoring variables
        self.score_req = SetMarkerPosition.Request()
        self.id = Int8()
        self.position = Point()

        self.marker_point = Point()

        self.marker_id = -1
        self.last_id = -1
        self.last_pos = Point()
        self.new_callback = False


  
        

    


    def timer_callback(self):

        self.send_request_scoring(self.id.data, self.position)
        self.get_logger().info(f" id: {self.id.data}, position: {self.position}")
        return 

    def marker_callback(self, msg):
      
        self.id.data = int(msg.data)
        self.get_logger().info(f"{msg.data}")

        if self.last_id != msg.marker_ids: 
            self.marker_point = msg.poses.position
            self.marker_ids = msg.marker_ids
            self.last_id = msg.marker_ids
            self.send_request_scoring(self.marker_ids, msg.poses.position)

            self.get_logger().info(f"{msg.marker_ids}")
            self.get_logger().info(f"X: {self.marker_point.x} Y. {self.marker_point.y}")
            self.new_callback = False

    def marker_poses(self, msg):
        self.get_logger().info(str(msg.position))
        self.get_logger().info(f"setting pose: {msg.position}")
        self.position = msg.position

    def send_request_scoring(self, marker_id, marker_pos):
        self.score_req.marker_id = int(marker_id)
        self.score_req.marker_position = marker_pos
        self.get_logger().info(f"sending request: id: {marker_id}, position: {marker_pos}")
        
        self.future = self.scoring_client.call_async(self.score_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"REQUEST FINISHED")
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    controller = new_aruco()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
