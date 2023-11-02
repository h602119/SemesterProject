import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from bug2_interfaces.action import Bug2Action
from bug2_interfaces.srv import NinePoint
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


class RobotActionClient(Node):

    def __init__(self):
        super().__init__('robot_client')
        self._action_client = ActionClient(self, Bug2Action, 'action_robot')

        self.nine_p_client = self.create_client(NinePoint, 'nine_point')
        while not self.nine_p_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for wall follower...')

    def send_goal(self):
        self.nine_point_request()

        return
    
        user_input = input("Please input GO-TO Coordinates ... ")
        array = user_input.split(',')
        goal_msg = Bug2Action.Goal()
        try:
            goal_msg.target_position.x = float(array[0])
            goal_msg.target_position.y = float(array[1])
        except ValueError:
            goal_msg.target_position.x = 3.0
            goal_msg.target_position.y = 0.0

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def nine_point_request(self):

        NinePoint.Request().create_point = True
        self.future = self.nine_p_client.call_async(NinePoint.Request())
        rclpy.spin_until_future_complete(self, self.future)

        result = self.future.result()

        list = result.data_x
        print(list)

def main(args=None):
    rclpy.init(args=args)

    action_client = RobotActionClient()

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()