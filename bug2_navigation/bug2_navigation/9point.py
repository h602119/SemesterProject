import rclpy
from rclpy.node import Node
import math

from bug2_interfaces.srv import NinePoint
from std_msgs.msg import Float64
import numpy as np

import time

class Point_9(Node):
    def __init__(self):
        super().__init__("Bug2_Controller")
        self.get_logger().info("Bug2_Controller Node created")


        self.srv = self.create_service(NinePoint, 'nine_point', self.nine_point_service)


    def nine_point_service(self, request, response):

        test = 1.0

        print(type(response.data_x))
        print(response.data_x)
        
        print(f'Test: {type(test)}')
        print(test)
        
        
        if(request.create_point):

            NinePoint.Response().data_x = test

            return response

    

def main(args=None):

    rclpy.init(args=args)

    controller = Point_9()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
