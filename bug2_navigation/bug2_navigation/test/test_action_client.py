import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from bug2_interfaces.action import Bug2Action


class TestActionClient(Node):

    def __init__(self):
        super().__init__('test_action_client')
        self._action_client = ActionClient(self, Bug2Action, 'testrobot')

    def send_goal(self):
        
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



def main(args=None):
    rclpy.init(args=args)

    action_client = TestActionClient()

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()