import rclpy
from rclpy.node import Node

from bug2_interfaces.srv import GoToPoint


class Bug2(Node):
    def __init__(self):
        super().__init__("Bug2_Controller")
        self.get_logger().info("Bug2_Controller Node created")
        
        self.gtp_client = self.create_client(GoToPoint, 'go_to_point')
        

        
        #Go to point variables:
        self.req = GoToPoint.Request()
        self.res = GoToPoint.Response()
        self.res.success = False
        self.gtp_move = True
        self.gtp_wall_found = False

        #Variables related to calculating the closest pos
        self.closest_pos = [0, 0]
        self.position = 0
        self.current_angle = 0

    def send_request_go_to_point(self, move, x, y):
        print('Calling go to point async')
        self.req.move_switch = move
        self.req.target_position.x = x
        self.req.target_position.y = y

        self.future = self.gtp_client.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)
        
        result = self.future.result()
        print(result.success)
        return result

    

def main(args=None):
    rclpy.init(args=args)

    client = Bug2()

    response = client.send_request_go_to_point(True, 3.0, 0.0)

    client.get_logger().info(f'{response.success}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
