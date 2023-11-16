import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from threading import Thread

from bug2_interfaces.action import Bug2Action
from bug2_interfaces.srv import NinePoint
import time

class RobotActionClient(Node):

    def __init__(self, namespace):
        super().__init__('robot_client_' + namespace)

        self.get_logger().info(f"Robot controller algorithm started for {namespace}")

        action_topic = f'/{namespace}/action_robot'
        self._action_client = ActionClient(self, Bug2Action, action_topic)


        self.nine_p_client = self.create_client(NinePoint, 'nine_point')
        while not self.nine_p_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for 9Point service in {namespace}...')

        self.req = NinePoint.Request()

    def send_goal(self):
        array = self.nine_point_request()

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
    
    def urgent_point_located(self):
        pass

    def check_robots_location(self):
        pass
    


def run_client(namespace):
    action_client = RobotActionClient(namespace)
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 context here

    thread_bug0 = Thread(target=run_client, args=('tb3_0',))
    thread_bug1 = Thread(target=run_client, args=('tb3_1',))

    thread_bug0.start()
    
    #Sleep for 5 seconds so that the robots do not start at the exact same time.
    time.sleep(5)

    thread_bug1.start()

    thread_bug0.join()
    thread_bug1.join()

    rclpy.shutdown()  # Shutdown the ROS 2 context after threads complete

if __name__ == '__main__':
    main()