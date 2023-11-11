import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class WallFollower(Node):
    def __init__(self):
        super().__init__("Wall_Follower")
        self.get_logger().info("Wall_follower Node created")
        self.sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)

        self.srv = self.create_service(SetBool, 'wall_follower', self.wall_follower_service)

        #Default values for the lidar variables as a placeholder until the actual sensor values are recieved through from the ros topic
        self.lidar_front_direct = 100
        self.lidar_left_front = 100
        self.lidar_left_direct = 100
        self.lidar_right_direct = 100
        self.lidar_right_front = 100
        
        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 


        #Boolean to trigger if a wall is found
        self.is_wall_found = False

        #Boolean to trigger if there is a corner
        self.is_corner = False


        #String representation of which mode is activated
        """
        go          ==> drive()
        follow wall ==> follow_wall()
        full stop   ==> full_stop()
        neutral     ==> return
        """ 
        self.drive_mode = "full_stop"

        #global distance checks
        self.distance_wall_ahead = 0.6

        #global velocity speeds
        self.velocity_fast = 0.5
        self.velocity_normal = 0.3
        self.velocity_slow = 0.15
        self.velocity_very_slow = 0.05
        
        #global rotation speeds
        self.rotation_very_fast = 0.6
        self.rotation_fast = 0.3
        self.rotation_normal = 0.2
        self.rotation_slow = 0.1
        self.rotation_very_slow = 0.05

        self.go_to_x = 0
        self.go_to_y = 0

        self.position = 0
        self.stop = False
        
    def wall_follower_service(self, request, response):
        
        if request.data:
            self.drive_mode = "follow wall"
            response.success = True
            self.stop = False
        else:
            response.success = True
            self.drive_mode = "neutral"
            response.message = ""

        if response.message == "stop":
            print('stop message received')
            self.stop = True
            self.drive_mode = "full stop"

            
        
        response.message = str(round(self.position.x,3))+","+str(round(self.position.y,3))
        
        return response


    #Callback function for the Turtlebots Lidar topic /scan
    def clbk_laser(self, msg):
        decimal = 2
        self.lidar_front_direct = round(msg.ranges[0], decimal)

        self.lidar_left_front = round(msg.ranges[45], decimal)
        self.lidar_left_direct = round(msg.ranges[90], decimal)

        self.lidar_right_direct = round(msg.ranges[270], decimal)
        self.lidar_right_front = round(msg.ranges[315], decimal)
        

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
            full_stop(self)
            return

        if not self.is_wall_found:
            wall_check(self)

        #Perform a given task based on which drive mode (command) is current
        mode = self.drive_mode

        if mode == "go":
            drive(self)

        elif mode == "follow wall":
            follow_wall(self)

        elif mode == "full stop":
            print('no input so stop')
            full_stop(self)
        else:
            return

def wall_check(self):
    #Check if there is a wall ahead

    if self.lidar_front_direct <= self.distance_wall_ahead:
        self.is_wall_found = True
        full_stop(self)
        self.drive_mode = "follow wall"
        return True

def full_stop(self):
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    self.pub.publish(msg)

def drive(self):
        msg = Twist()
        msg.linear.x = self.velocity_normal
        msg.angular.z = 0.0
        self.pub.publish(msg)

def follow_wall(self):
    if not self.is_wall_found:
        self.drive_mode = "go"
        return
    msg = Twist()
    msg.linear.x = 0.0

    #from -1 to 1 (-1 rotates towards right)
    msg.angular.z = 0.0

    distance  = self.distance_wall_ahead


    if self.lidar_front_direct < distance:
        print("1")
        msg.linear.x = 0.01
        msg.angular.z = -self.rotation_fast

    elif (self.lidar_front_direct > self.lidar_left_front) and (self.lidar_left_direct > self.lidar_left_front) and self.lidar_left_front < distance:
        print("2")
        msg.linear.x = 0.0
        msg.angular.z = -self.rotation_fast

    elif (self.lidar_left_direct > self.lidar_front_direct) and (self.lidar_left_direct < distance):
        print("3")
        msg.linear.x = 0.0
        msg.angular.z = -self.rotation_fast
    
    elif self.lidar_left_front < distance*0.7:
        print("4")
        msg.linear.x = self.rotation_slow
        msg.angular.z = -self.rotation_fast
    
    elif (self.lidar_left_direct > 0.75 * distance) and (self.lidar_left_front > distance):
        print("5")
        msg.linear.x = self.velocity_normal
        msg.angular.z = self.rotation_fast
        self.is_corner = True
    
    elif (self.lidar_left_direct > distance) or (self.lidar_left_front > distance):
        print("6 correcting")
        msg.linear.x = self.velocity_slow
        msg.angular.z = self.rotation_fast

    else:
        print("DEFAULT")
        msg.linear.x = self.velocity_normal
        msg.angular.z = self.rotation_very_slow
    

    #If its rounding a corner
    if self.is_corner:
        if (self.lidar_left_direct > self.lidar_left_front) and (self.lidar_front_direct > self.lidar_left_front):
            print(" 2 -> 1 Breaking out of sharp cornering")
            self.is_corner = False
        else:
            print(" 2 -> 2 Sharp cornering")
            msg.linear.x = self.velocity_slow
            msg.angular.z = self.rotation_very_fast


    #GEN 1
        """
        if self.lidar_front_direct < distance:
            msg.linear.x = 0.0
            msg.angular.z = -(self.rotation_fast)
            
        elif self.lidar_left_direct < distance * 0.5:
            msg.linear.x = self.velocity_normal
            msg.angular.z = -(self.rotation_fast)

        elif self.lidar_left_front < distance*0.6:
            msg.linear.x = 0.0
            msg.angular.z = -(self.rotation_fast)

        elif (self.lidar_left_front > distance) and (self.lidar_left_direct > distance):
            msg.linear.x = self.velocity_slow
            msg.angular.z = self.rotation_fast
        elif (self.lidar_left_front > distance * 2) and (self.lidar_left_direct > distance * 2):
            msg.linear.x = self.velocity_very_slow
            msg.angular.z = self.rotation_normal

        elif (self.lidar_left_direct >= 0.5 * distance) and (self.lidar_left_direct <= distance):
            msg.linear.x = self.velocity_fast
            msg.angular.z = 0.0

        else:
            msg.linear.x = self.velocity_normal
            msg.angular.z = self.rotation_slow
"""

    self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    controller = WallFollower()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
