import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from threading import Thread
from std_srvs.srv import SetBool
from bug2_interfaces.action import Bug2Action
from bug2_interfaces.srv import NinePoint
from bug2_interfaces.srv import UrgentPoint
from nav_msgs.msg import Odometry
import time
import math

class RobotActionClient(Node):

    def __init__(self, namespace):
        super().__init__('robot_client_' + namespace)

        self.get_logger().info(f"Robot controller algorithm started for {namespace}")

        action_topic = f'{namespace}/action_robot'
        self._action_client = ActionClient(self, Bug2Action, action_topic)

        self.service = self.create_service(SetBool, '/'+namespace+'/request_points', self.handle_request_points)

        self.nine_p_client = self.create_client(NinePoint, 'nine_point')
        while not self.nine_p_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for 9Point service in {namespace}...')

        self.urgent_point_service = self.create_service(UrgentPoint, '/'+namespace+'/urgent_point', self.handle_urgent_point)

        self.big_fire_client = self.create_client(SetBool, '/'+namespace+'/big_fire')

        self.odom_sub = self.create_subscription(Odometry, '/tb3_0/odom', self.callback_odom_0, 10)

        self.odom_sub = self.create_subscription(Odometry, '/tb3_1/odom', self.callback_odom_1, 10)

        # Determine the target namespace for the urgent point client
        target_namespace = 'tb3_1' if namespace == 'tb3_0' else 'tb3_0'

        self.urgent_point_client = self.create_client(UrgentPoint, '/'+target_namespace+'/urgent_point')

        self.req = NinePoint.Request()

        self.namespace = namespace

        self.x = -1.0
        self.y = -3.0

        #The robot that found the urgency.
        self.urgency_id = 4

        self.urgency_x = 0.0
        self.urgency_y = 0.0

        #The urgency id to notify the other robot.
        self.redirect_urgency = 999

        #Lists over ids and position
        self.id_table = set()

        #Location of bots (only for first thread)
        self.bug0_pos = None
        self.bug1_pos = None
        
        #A boolean to ensure it only sends points once.
        self.sending = False

        #If an urgent point is located sending coordinates for this point.
        self.urgent_request = False

        #Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if not self.namespace:
            return
        self.check_robots_location()
        self.get_logger().info(f"{self.namespace} timer callback")

      
    def send_drop_and_go(self):
        self.get_logger().info(f"{self.namespace} send drop and go")
        req = SetBool.Request()
        req.data = True

        self.urgent_request = True

        future = self.big_fire_client.call_async(req)
        


    def handle_urgent_point(self, request, response):
        self.get_logger().info(f"{self.namespace} handle urgent point")
        urgency_id = self.urgency_id

        robot_id = request.robot_id
        target_x = request.target_x
        target_y = request.target_y
        tag_id = request.tag_id

        if robot_id in self.id_table:
            return response
        
        self.id_table.add(tag_id)

        #The urgency is spotted by the other robot, and this robot have to move to the other robot.
        if tag_id == self.redirect_urgency:
            self.send_drop_and_go()
            response.received = True
            self.urgency_x = target_x
            self.urgency_y = target_y
            return response

        response.received = False
        if tag_id == urgency_id:
            response.received = True
            self.redirect_other_robot(robot_id, target_x, target_y, self.redirect_urgency)
            self.urgency_x = target_x
            self.urgency_y = target_y
        else:
            self.redirect_other_robot(robot_id, target_x, target_y, tag_id)
            

        return response 


    def redirect_other_robot(self, robot_id, target_x, target_y, tag_id):
        self.get_logger().info(f"{self.namespace} redirect other robot")
        req = UrgentPoint.Request()
        req.robot_id = robot_id
        req.target_x = target_x
        req.target_y = target_y
        req.tag_id = tag_id

        future = self.urgent_point_client.call_async(req)
    
        return future.result()



    def handle_request_points(self, request, response):
        self.get_logger().info(f"{self.namespace} handle request points")
        if request.data:  # If the request data is True
            # Logic to update points
            self.send_goal()

        else:
            response.success = False
            response.message = "Request data is False. No new points generated."

        return response      


    def send_goal(self):
        if not self.sending:
            self.sending = True
            self.get_logger().info(f"{self.namespace} send goal")
            #array = self.nine_point_request()
            
            array = [[-1.0], [-1.0]]

            if self.namespace == 'tb3_0':
                array = [[3.0], [-1.0]]

            if self.urgent_request:
                array = [[self.urgency_x], [self.urgency_y]]

            goal_msg = Bug2Action.Goal()
            goal_msg.positions_x = array[0]
            goal_msg.positions_y = array[1]

            self.get_logger().info('Received coordinates. Sending goal.')
            self.sending = False
            self._action_client.wait_for_server()
            return self._action_client.send_goal_async(goal_msg) 
        time.sleep(2)

    def nine_point_request(self):
        self.get_logger().info(f"{self.namespace} nine point request")
        self.req.create_point = True
        self.future = self.nine_p_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        result = self.future.result()
        return [result.data_x, result.data_y]


    """
        While not all points found run program. (5 in total, check that they are not the same)
        When all points found publish this, causing bug2 to close.
        Subscribe to new points located, update list on points found,
        if an urgent point is located act accordingly 
    """


    def send_move_on_request(self, namespace):
        self.get_logger().info(f"{self.namespace} send move on request")
        self.move_along_client = self.create_client(SetBool, '/'+namespace+'/move_along')

        self.urgent_request = False

        req = SetBool.Request()
        req.data = True

        future = self.move_along_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


    def check_robots_location(self):
        self.get_logger().info(f"{self.namespace} check robot location")
        if self.namespace != 'tb3_0':
            return

        if self.bug0_pos is None or self.bug1_pos is None:
            return

        # Calculate the distance between the two robots
        dx = self.bug0_pos.x - self.bug1_pos.x
        dy = self.bug0_pos.y - self.bug1_pos.y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if the distance is within the specified range
        if distance <= 0.4:
            # Both robots are close enough to each other
            self.send_move_on_request('tb3_0')
            self.send_move_on_request('tb3_1')
    


    def callback_odom_0(self, msg):
        self.bug0_pos = msg.pose.pose.position

    def callback_odom_1(self, msg):
        self.bug1_pos = msg.pose.pose.position


        

def run_client(namespace):
    # No need to initialize ROS context here as it's already initialized in main
    action_client = RobotActionClient(namespace)
    action_client.send_goal()

    rclpy.spin(action_client)
    action_client.destroy_node()

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 context here

    node_tb3_0 = RobotActionClient('tb3_0')
    node_tb3_0.send_goal()
    
    node_tb3_1 = RobotActionClient('tb3_1')
    node_tb3_1.send_goal()

    # Use MultiThreadedExecutor to handle callbacks for both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_tb3_0)
    executor.add_node(node_tb3_1)

    try:
        executor.spin()  # This will handle callbacks for both nodes
    finally:
        executor.shutdown()
        node_tb3_0.destroy_node()
        node_tb3_1.destroy_node()
        rclpy.shutdown()  # Shutdown the ROS 2 context

if __name__ == '__main__':
    main()
