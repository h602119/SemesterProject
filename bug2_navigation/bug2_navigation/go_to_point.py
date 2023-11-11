import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from bug2_interfaces.srv import GoToPoint

class GoToPointNode(Node):

    def __init__(self):
        super().__init__("Go_To_Point")
        self.get_logger().info("go_to_point node created")
        self.sub = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.srv = self.create_service(GoToPoint, 'go_to_point', self.gtp_service)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Boolean if a position is given
        self.is_pos_given = False
        self.service_pos_given = False

        #Boolean if the robot has reached the destination
        self.is_at_destination = False

        #Boolean if the robot is rotating
        self.is_rotating = False

        # Wall and LiDAR values
        self.is_wall_ahead = False
        self.wall_distance_check = 1
        self.wall_side = 0.2

        self.lidar_front_direct = 100
        self.lidar_left_front = 100
        self.lidar_right_front = 100

        #Global distance values
        self.distance = 0
        self.current_distance = 0

        #Global pos coordinates.
        self.position = 0
        self.go_to_X = 0
        self.go_to_Y = 0
        self.go_to_angle = 0

        self.req = GoToPoint.Request()
        self.res = GoToPoint.Response()

        self.stop = True

        self.target_locked = False


    def gtp_service(self, request, response):
        
        print(f'X: {request.target_position.x}, Y: {request.target_position.y}')

        if (request.move_switch) and (not self.is_at_destination) and (not self.is_wall_ahead):
            if not self.target_locked:
                set_point(self, request.target_position.x, request.target_position.y)
            response.success = True
            self.stop = False
            self.target_locked = True
        elif self.is_wall_ahead:
            response.success = False
            
        elif not request.move_switch:
            print('stop request')
            full_stop(self)
            self.stop = True
            response.success = True
            self.target_locked = False
        else:
            response.success = True

        return response
            

    def clbk_laser(self, msg):
        decimal = 3
        self.lidar_front_direct = round(msg.ranges[0], decimal)
        self.lidar_left_front = round(msg.ranges[60], decimal)
        self.lidar_right_front = round(msg.ranges[300], decimal)


    def callback_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]


    def timer_callback(self):
        
        if self.stop:
            return

        destination_check(self)

        if self.is_at_destination:
            #If the destination is reached, reset the goal
            #reset_destination(self)
            return
        elif self.is_wall_ahead:
            wall_check(self)

        else:
            #correct and drive until the destination is reached
            correction(self)
            if not self.is_rotating:
                move(self)
            
            wall_check(self)
        


"""def set_point(self):
        
    #Get input X and Y coordinates from user and set global variables.
    print(f"The bot is currently at pos x:{round(self.position.x,5)}, y:{round(self.position.y,5)}")
    user_input = input("Please input GO-TO Coordinates ... ")
    coordinates = user_input.split(',')
    try:
        self.go_to_X = float(coordinates[0])
        self.go_to_Y = float(coordinates[1])
        self.is_pos_given = True
        self.is_at_destination = False

        #Check if the position given is the same as user position
        distance = distance_check(self)
        self.distance = distance
        destination_check(self)

    except ValueError:
        print("INPUT ERROR, type coordiantes as such: num1, num2 ")
        reset_destination(self)
"""


def set_point(self, x, y):
        
        self.go_to_X = x
        self.go_to_Y = y

        if self.go_to_X != x or self.go_to_Y != y:
            reset_destination(self)

        self.is_pos_given = True
        self.is_at_destination = False

        distance = distance_check(self)
        self.distance = distance
        destination_check(self)
        locate_point(self)
        correction(self)

def locate_point(self): 

    # calculate angle to rotate to.
    # and then drive until bot is positioned the same as the coordinates given by user. 

    diff_x = self.go_to_X - self.position.x
    diff_y = self.go_to_Y - self.position.y

    #atan2 gives a range from -PI to PI which is the same as YAW.
    self.go_to_angle = math.atan2(diff_y, diff_x)          

    print(f"The angle is : {round(self.go_to_angle,5)}")
    print(f"Current YAW : {round(self.yaw,5)}")

def distance_check(self):
    #Rounding since with my fail margin this is more than precice enough
    pos_x = round(self.position.x, 5)
    pos_y = round(self.position.y, 5)
    go_to_x = round(self.go_to_X, 5)
    go_to_y = round(self.go_to_Y, 5)

    distance = math.sqrt(pow(go_to_x-pos_x, 2) + pow(go_to_y-pos_y, 2))
    return distance

def destination_check(self):
    
    distance = distance_check(self)

    #decide if the bot is an accepted distance from the goal.
    if (distance <= 0.2) and (distance >= -0.2):
        self.is_at_destination = True
        print('distance stop')
        full_stop(self)
        print(f"Destination reached: {distance} from goal...")

def correction(self):
    msg = Twist()
    msg.linear.x = 0.0
    #from -1 to 1 (-1 rotates towards right)
    msg.angular.z = 0.0
    yaw = self.yaw
    angle = self.go_to_angle

    
    #TODO find a range thats efficient and solve the problem of overshooting
    #checking if angle is within a +/- 6 degrees (0.1radians) range
    range = 0.1
    orientation_difference = angle - yaw

    # Determine the direction of rotation based on the shortest path
    if orientation_difference > math.pi:
        orientation_difference -= 2 * math.pi
    elif orientation_difference < -math.pi:
        orientation_difference += 2 * math.pi

    if abs(orientation_difference) <= range:
        self.is_rotating = False

        full_stop(self)
    else:
        self.is_rotating = True

        # Determine the direction of rotation based on the sign of orientation_difference
        if orientation_difference > 0:
            msg.angular.z = 0.2
        else:
            msg.angular.z = -0.2
    
    self.pub.publish(msg)

def move(self):
    msg = Twist()
    self.current_distance = distance_check(self)
   # print(f"Distance  = {self.distance}, Current distance = {self.current_distance}")

    #We're working with incredibly precice numbers, moving at not perfect speeds, rounding helps with achieving the wanted result.
    if (round(self.current_distance,3) > round(self.distance+0.2,3)) or (round(self.current_distance,3) <= (round(self.distance/3,3))):
        print("-------------- Updating Distance_check ------------------")
        self.distance = distance_check(self)
        locate_point(self)
        msg.angular.z = 0.0
        msg.linear.x = 0.0
    else:
        msg.angular.z = 0.0
        msg.linear.x = 0.25
    
    self.pub.publish(msg)

def wall_check(self):
    if (self.lidar_front_direct < self.wall_distance_check) or (self.lidar_left_front < self.wall_side) or (self.lidar_right_front < self.wall_side):
        if not self.is_wall_ahead and self.lidar_front_direct < self.wall_side:
            full_stop(self)
            print('Wall check stop')

        self.is_wall_ahead = True
        
    else:
        self.is_wall_ahead = False

def full_stop(self):
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    
    self.pub.publish(msg)

def reset_destination(self):
    #Reset all values back to original
    self.is_pos_given = False
    self.is_at_destination = False
    self.is_rotating = False
    self.position = 0
    self.go_to_X = 0
    self.go_to_Y = 0
    self.go_to_angle = 0

def main(args=None):
    rclpy.init(args=args)

    go_to_point = GoToPointNode()

    rclpy.spin(go_to_point)
        
    go_to_point.destroy_node()    

    rclpy.shutdown()

if __name__ == '__main__':
    main()
