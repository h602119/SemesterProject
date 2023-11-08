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
            self.get_logger().info('waiting for 9Point...')


        self.req = NinePoint.Request()
        self.res = NinePoint.Response()


    def send_goal(self):
        array = self.nine_point_request()
    
        goal_msg = Bug2Action.Goal()
        
        goal_msg.positions_x = array[0]
        goal_msg.positions_y = array[1]
        
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def nine_point_request(self):

        self.req.create_point = True
        self.future = self.nine_p_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        result = self.future.result()

        print(f'Result x: {result.data_x}')
        print(f'Result y: {result.data_y}')
        return [result.data_x, result.data_y]
   

def main(args=None):
    rclpy.init(args=args)

    action_client = RobotActionClient()
    
    future = action_client.send_goal()
    
    rclpy.spin_until_future_complete(action_client, future)
    

if __name__ == '__main__':
    main()