import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class Bug2(Node):
    def __init__(self):
        super().__init__("Bug2_Controller")
        self.get_logger().info("Bug2_Controller Node created")

        self.wf_client = self.create_client(SetBool, 'wall_follower')
        while not self.wf_client.wait_for_service(timeout_sec=5.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        
        #Wall_follower variables:
        self.req = SetBool.Request()
        self.res = SetBool.Response()
           


    def send_request_wall_follower(self, move):
        
            print('Calling wall_follower async')
            self.req.data = move

            self.future = self.wf_client.call_async(self.req)
        
            
            rclpy.spin_until_future_complete(self, self.future)
            print('SPIN complete')

            result = self.future.result()
            
            print(result.message)
            
            return result
            

def main(args=None):
    rclpy.init(args=args)

    client = Bug2()

    response = client.send_request_wall_follower(True)

    client.get_logger().info(f'{response.success}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
