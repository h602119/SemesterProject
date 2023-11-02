import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 
import math
import random 
#TODO: check for walls


def distance_between_two_points(p1, p2):
        #print("POINTS: ", p1, p2)
        x_len = p1.x - p2.x
        y_len = p1.y - p2.y
        
        hyp = math.sqrt( pow(x_len,2) +  pow(y_len,2)) 
        #print("distance: ", hyp)
        return hyp

class nine_points(Node):

    def __init__(self):
        super().__init__('nine_points')
        self.past_points = []
        new_path = self.generate_points()
        print("ended")
        

    

    def generate_points(self):
        
        point_list = []
        width = 20 
        number_of_points = 9 
        randomness = 0.2 
        half_rand = randomness/2
        too_close = 0.05 # this sets the maximum accepted distance between points
        point_line = math.floor(math.sqrt(number_of_points))
        distance_pr_point = width / point_line
        padding = distance_pr_point/2
        formula = -(width/2) + padding
        flip = -1 
        
        for point_x in range(point_line): 
            flip*=-1
            for point_y in range(point_line):

                tmp_point = Point()
                random_float = random.uniform(-half_rand, half_rand)
                print("random float1: ",random_float)
                tmp_point.x = point_x*distance_pr_point + random_float + formula

                random_float = random.uniform(-half_rand, half_rand)
                pprint("random float2: ",random_float)
                tmp_point.y = (point_y*distance_pr_point + random_float + formula)*flip

                #pick a new point untill its far enough away from other points
                while any(distance_between_two_points(ele,tmp_point) < too_close for ele in self.past_points): # add check to see if element is on wall 

                    
                    random_float = random.uniform(-half_rand, half_rand)
                    print("random float1: ",random_float)
                    tmp_point.x = point_x*distance_pr_point + random_float + formula

                    random_float = random.uniform(-half_rand, half_rand) 
                    print("random float2: ",random_float)
                    tmp_point.y = (point_y*distance_pr_point + random_float + formula)*flip

                    print(tmp_point.x, tmp_point.y)
                
                print("new point beeing added: ",tmp_point)

                point_list.append(tmp_point)
                self.past_points.append(tmp_point)
        

        print(len(point_list))
        print(point_list)
        return point_list
    
       




 
        

        
            
def main(args=None):
    rclpy.init(args=args)

    nine_points_runner = nine_points()
    rclpy.spin(nine_points)
    nine_points.destroy_node()
    rclpy.shutdown()
 
 






if __name__ == '__main__':
    main()