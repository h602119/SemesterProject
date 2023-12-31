import rclpy
from rclpy.node import Node

import math

from std_srvs.srv import SetBool
from bug2_interfaces.srv import GoToPoint
from rclpy.action import ActionServer
from bug2_interfaces.action import Bug2Action
import time

class Bug2(Node):
    def __init__(self):
        super().__init__("Bug2_Controller")
        self.get_logger().info("Bug2_Controller Node created")
        
        self.wf_client = self.create_client(SetBool, 'wall_follower')
        while not self.wf_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for wall follower...')

        self.gtp_client = self.create_client(GoToPoint, 'go_to_point')
        while not self.gtp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for go to point...')

        self.action_server = ActionServer(self, Bug2Action, 'action_robot', self.action_callback)


        #from action client
        self.client_x = 0
        self.client_y = 0

        #Go to point variables:
        self.req = GoToPoint.Request()
        self.res = GoToPoint.Response()

        #Wall_follower variables:
        self.wf_req = SetBool.Request()
        self.wf_res = SetBool.Response()

        #Variables related to calculating the closest pos
        self.closest_pos = [0, 0]
        self.position_x = 0
        self.position_y = 0

        self.yaw = 0
        self.current_angle = 0

        #Central initial values
        self.gtp_x = 0
        self.gtp_y = 0
        self.starting_x = 0
        self.starting_y = 0
        self.target_angle = 0

        #Boolean
        self.initialise_variables = False
        self.is_on_line = False

        #action server variables
        self.goal_pos_x = 0.0
        self.goal_pos_y = 0.0
        self.action_received = False

    def action_callback(self, target):
        self.goal_pos_x = float(target.request.target_position.x)
        self.goal_pos_y = float(target.request.target_position.y)
        self.action_received = True
        self.get_logger().info('Executing goal...')
        self.get_logger().info(f'Position received : {target.request.target_position.x}, {target.request.target_position.y}')
        target.succeed()
        result = Bug2Action.Result()
        return result

    def send_request_go_to_point(self, move, x, y):
        if not self.action_received:
            return
        
        self.req.move_switch = move
        self.req.target_position.x = float(x)
        self.req.target_position.y = float(y)

        self.future = self.gtp_client.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        result = self.future.result()
        return result.success


    def send_request_wall_follower(self, move):

        if not self.action_received:
            return
            
        self.wf_req.data = move

        self.future = self.wf_client.call_async(self.wf_req)

        rclpy.spin_until_future_complete(self, self.future)

        result = self.future.result()

        message = result.message

        numbers = message.split(',')
        self.position_x = float(numbers[0])
        self.position_y = float(numbers[1])

        return result       


    def set_variables(self, x, y):
        if not self.initialise_variables:

            self.wf_req.data = False
    
            self.future = self.wf_client.call_async(self.wf_req)

            rclpy.spin_until_future_complete(self, self.future)
  
            result = self.future.result()

            message = result.message
            numbers = message.split(',')

            self.gtp_x = x
            self.gtp_y = y
            self.starting_x = self.position_x = self.closest_pos[0] = float(numbers[0])
            self.starting_y = self.position_y = self.closest_pos[1] = float(numbers[1])

            self.target_angle = self.calculate_degrees(self.starting_x, self.starting_y, self.gtp_x, self.gtp_y)
            self.initialise_variables = True


    def calculate_degrees(self, pos_x, pos_y, goal_x, goal_y):

        delta_x = goal_x - pos_x
        delta_y = goal_y - pos_y

        angle_radians = math.atan2(delta_y, delta_x)

        angle_degrees = round(math.degrees(angle_radians),3)

        return angle_degrees


    def angle_check(self, angle_self, angle_goal):
        difference = math.sqrt(math.pow(angle_self-angle_goal,2))

        if difference < 5:
            return True
        else:
            return False


    def closest_point(self, current_x, current_y, prev_x, prev_y, goal_x, goal_y):
        curr_diff_x = math.sqrt(math.pow(current_x-goal_x, 2))
        curr_diff_y = math.sqrt(math.pow(current_y-goal_y, 2))
        curr_diff = math.sqrt(math.pow(curr_diff_x+curr_diff_y, 2))

        prev_diff_x = math.sqrt(math.pow(prev_x-goal_x, 2))
        prev_diff_y = math.sqrt(math.pow(prev_y-goal_y, 2))
        prev_diff = math.sqrt(math.pow(prev_diff_x+prev_diff_y, 2))


        diff_diff = (curr_diff - prev_diff) < -0.5

        if curr_diff < prev_diff and diff_diff:
            self.closest_pos[0] = current_x
            self.closest_pos[1] = current_y
            
            return True
        else:
            return False


    def update_closest_point(self):
        return self.closest_point(self.position_x, self.position_y, self.closest_pos[0], self.closest_pos[1], self.gtp_x, self.gtp_y)


    def calculate_and_check(self):
        target_angle = self.target_angle
        current_angle = self.calculate_degrees(self.position_x, self.position_y, self.gtp_x, self.gtp_y)

        if self.angle_check(current_angle, target_angle):
            return True
        
        else:
            return False


    def goal_reached(self):
        pos_x = round(self.position_x, 5)
        pos_y = round(self.position_y, 5)
        go_to_x = round(self.gtp_x, 5)
        go_to_y = round(self.gtp_y, 5)

        distance = math.sqrt(pow(go_to_x-pos_x, 2) + pow(go_to_y-pos_y, 2))
        if (distance <= 0.2) and (distance >= -0.2):
            return True
        else:
            return False

    

def main(args=None):

    rclpy.init(args=args)

    client = Bug2()


    #Go To point variables
    wall_found = False
    x = 4.0
    y = 4.0
    while not client.action_received:
        client.get_logger().info('Awaiting position')
        rclpy.spin_once(client)
        x = float(client.goal_pos_x)
        y = float(client.goal_pos_y)

    client.get_logger().info(f'Navigation towards : {x}, {y}')

    #Wall follower variables
    wf_drive = False
    
    #Goal
    goal_reached = False
    
    

    client.set_variables(x, y)

    while(not goal_reached):
        

        wf_drive = False
        client.wf_res.message = ""
        
        if not wall_found:
            gtp_response = client.send_request_go_to_point(True, x, y)
            
            if not gtp_response:
                client.get_logger().info('Switching mode: FW')
                wall_found = True
            else:
                time.sleep(1)
        else:
            wf_drive = True
        
        
        client.send_request_wall_follower(wf_drive)
            
        

        if client.calculate_and_check():
            
            if client.update_closest_point():
                client.get_logger().info('CLOSER POINT LOCATED')
                if wf_drive:
                    client.get_logger().info('Switching mode: GTP')
                    client.wf_res.message = "full stop"
                    wf_response = client.send_request_wall_follower(False)
                    wall_found = False
                    


        if client.goal_reached():
            
            goal_reached = True
            gtp_response = client.send_request_go_to_point(False, x, y)
            client.wf_res.message = "full stop"
            wf_response = client.send_request_wall_follower(False)
            client.get_logger().info(f'Goal reached at : {client.position_x}, {client.position_y}')


    client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
