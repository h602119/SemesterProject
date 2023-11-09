import math
import rclpy
from rclpy.node import Node
from bug2_interfaces.srv import GoToPoint

class GoToPointNode(Node):

    def __init__(self):
        super().__init__("Go_To_Point")
        self.get_logger().info("go_to_point node created")

        self.srv = self.create_service(GoToPoint, 'go_to_point', self.gtp_service)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.req = GoToPoint.Request()
        self.res = GoToPoint.Response()


    def gtp_service(self, request, response):
        
        print(f'X: {request.target_position.x}, Y: {request.target_position.y}')

        if (request.move_switch):
            response.success = True
        else:
            response.success = False

        return response
            

    def timer_callback(self):
        
        print('.')
       
           

def main(args=None):
    rclpy.init(args=args)

    go_to_point = GoToPointNode()

    rclpy.spin(go_to_point)
        
    go_to_point.destroy_node()    

    rclpy.shutdown()

if __name__ == '__main__':
    main()
