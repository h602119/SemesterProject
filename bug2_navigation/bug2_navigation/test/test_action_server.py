import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from bug2_interfaces.action import Bug2Action


class TestActionServer(Node):

    def __init__(self):
        super().__init__('test_action_server')
        self._action_server = ActionServer(self, Bug2Action, 'testrobot', self.execute_callback)
        self.pos = 0
        self.bool = False

    def execute_callback(self, goal_handle):
        self.pos = goal_handle.request.target_position
        self.bool = True
        self.get_logger().info('Executing goal...')
        print(f'POS: {goal_handle.request.target_position.x}, {goal_handle.request.target_position.y}')
        goal_handle.succeed()
        result = Bug2Action.Result()
        return result
    
    def print_sum(self):
        print(self.pos.x + self.pos.y)


def main(args=None):
    message = False
    rclpy.init(args=args)
    
    test_action_server = TestActionServer()
    
    rclpy.spin_once(test_action_server)
    test_action_server.print_sum()
    message = test_action_server.bool
    
    print('hello'+str(message))

if __name__ == '__main__':
    main()