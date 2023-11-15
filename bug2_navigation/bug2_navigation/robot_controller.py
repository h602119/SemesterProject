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

        action_topic = f'/{namespace}/action_robot'
        self._action_client = ActionClient(self, Bug2Action, action_topic)

        self.service = self.create_service(SetBool, namespace+'/request_points', self.handle_request_points)

        self.nine_p_client = self.create_client(NinePoint, 'nine_point')
        while not self.nine_p_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for 9Point service in {namespace}...')

        self.urgent_point_service = self.create_service(UrgentPoint, namespace+'/urgentpoint', self.handle_urgent_point)

        self.big_fire_client = self.create_client(SetBool, namespace+'/big_fire')

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

        #The urgency id to notify the other robot.
        self.redirect_urgency = 999

        #Lists over ids and position
        self.id_table = set()

        #Location of bots (only for first thread)
        self.bug0_pos = None
        self.bug1_pos = None
        

    def send_drop_and_go(self):
        req = SetBool.Request()
        req.data = True

        future = self.urgent_point_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


    def handle_urgent_point(self, request, response):
        urgency_id = self.ugency_id

        robot_id = request.robot_id
        target_x = request.target_x
        target_y = request.target_y
        tag_id = request.target_id

        if robot_id in self.id_table:
            return response
        
        self.id_table.add(robot_id)

        #The urgency is spotted by the other robot, and this robot have to move to the other robot.
        if tag_id == self.redirect_urgency:
            self.send_drop_and_go()
            response.received = True
            return response

        response.received = False
        if tag_id == urgency_id:
            response.received = True
            self.redirect_other_robot(self.redirect_urgency, target_x, target_y)
        
        else:
            self.redirect_other_robot(robot_id, target_x, target_y)
            
        return response 


    def redirect_other_robot(self, robot_id, target_x, target_y):
        req = UrgentPoint.Request()
        req.robot_id = robot_id
        req.target_x = target_x
        req.target_y = target_y
        req.tag_id = self.redirect_urgency

        future = self.urgent_point_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


    def handle_request_points(self, request, response):
        if request.data:  # If the request data is True
            # Logic to update points
            self.send_goal()

        else:
            response.success = False
            response.message = "Request data is False. No new points generated."

        return response      


    def send_goal(self):
        #array = self.nine_point_request()
        
        array = [[-1.0, -1.0, -1.0, -1.0], [-2.0, -1.0, 0.0, 1.0]]

        goal_msg = Bug2Action.Goal()
        goal_msg.positions_x = array[0]
        goal_msg.positions_y = array[1]

        self.get_logger().info('Received coordinates. Sending goal.')

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


    def nine_point_request(self):
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

    
    def urgent_point_located(self, location_x, location_y):
        pass


    def send_move_on_request(self, namespace):
        self.move_along_client = self.create_client(SetBool, namespace+'/move_along')

        req = SetBool.Request()
        req.data = True

        future = self.move_along_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


    def check_robots_location(self):
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
    

    def run(self):
    # Initially send a goal
        self.send_goal()

        while rclpy.ok():
            self.get_logger().info(f"{self.namespace} waiting for new point requests...")
            # Instead of calling send_goal in a loop, just keep the node running
            # and waiting for requests from the SetBool service
            rclpy.spin_once(self)
            time.sleep(1)


    def callback_odom_0(self, msg):
        self.bug0_pos = msg.pose.pose.position


    def callback_odom_1(self, msg):
        self.bug1_pos = msg.pose.pose.position


def run_client(namespace):
    rclpy.init(args=None)  # Initialize a separate ROS 2 context for each thread

    action_client = RobotActionClient(namespace)
    action_client.run()

    action_client.destroy_node()
    rclpy.shutdown()


def main(args=None):
    # Thread for tb3_0
    thread_bug0 = Thread(target=run_client, args=('tb3_0',))
    thread_bug0.start()

    # Sleep to stagger the start times of the robots
    time.sleep(5)

    #Thread for tb3_1
    #thread_bug1 = Thread(target=run_client, args=('tb3_1',))
    #thread_bug1.start()

    # Join the threads to ensure they complete execution
    thread_bug0.join()
    #thread_bug1.join()


if __name__ == '__main__':
    main()