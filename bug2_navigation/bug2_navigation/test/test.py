import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class Test(Node):

    def __init__(self):
        super().__init__("Wall_Follower")
        self.get_logger().info("Wall_follower Node created")

        self.srv = self.create_service(SetBool, 'wall_follower', self.wf_service)

        #Position 
        self.pos_x = 10.0
        self.pos_y = 3.5
        
        self.req = SetBool.Request()
        self.res = SetBool.Response()
        

    def wf_service(self, request, response):
        print(f'DATA : {request.data}, RESPONSE SUCCESS : {response.success}, RESPONSE MESSAGE : {response.message}')
        print('Setting success and message')
        
        response.success = True
        response.message = "Testing"

        print(f'right before return:::{request}, {response}')
        return response
    

def main(args=None):
    rclpy.init(args=args)

    bug2_controller = Test()

    rclpy.spin(bug2_controller)

    bug2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
