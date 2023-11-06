import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 
import math
import random
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header 

import numpy as np
import matplotlib.pyplot as plt
#TODO: check for walls






# Sample occupancy grid matrix (replace this with your actual grid data)


# Create a binary mask to separate occupied and free cells




# Show the plot
plt.show()

class nine_points_algo_v1(Node):

    def __init__(self):
        super().__init__('nine_points')

      
        

        # Subscribe to the /map topic
        map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.past_points = []
        self.new_path = self.generate_points()
        
        x, y = self.point_to_arrays(self.new_path)

        #print(x, y)
        print("ended")
        

    def map_callback(self, msg):
        # Handle the received map data
        #print(msg.data)
        print(msg.data.count(0))
        print(msg.data.count(100))
        print(msg.data.count(-1))
        print(len(msg.data))
        print(msg.info.height)
        print(msg.info.width)
        print(msg.info.resolution)
        self.visualize_map(msg)

    def visualize_map(self, map_msg):
    # Extract the occupancy grid data
        occupancy_grid_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # Create a binary mask to separate occupied and free cells
        occupied_cells = occupancy_grid_data >= 50  # You can adjust this threshold
        free_cells = occupancy_grid_data < 50

        # Create a figure and axis
        fig, ax = plt.subplots()

        # Plot occupied cells in black and free cells in white
        ax.imshow(occupied_cells, cmap='gray', interpolation='none', extent=[0, map_msg.info.width, 0, map_msg.info.height])
        ax.imshow(free_cells, cmap='binary', interpolation='none', extent=[0, map_msg.info.width, 0, map_msg.info.height])

        # Set labels and title
        ax.set_xlabel('X (Cells)')
        ax.set_ylabel('Y (Cells)')
        ax.set_title('Occupancy Grid')

        # Show the plot
        plt.show()

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
                print("random float2: ",random_float)
                tmp_point.y = (point_y*distance_pr_point + random_float + formula)*flip

                #pick a new point untill its far enough away from other points
                while any(self.distance_between_two_points(ele,tmp_point) < too_close for ele in self.past_points): # add check to see if element is on wall 

                    
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
        #print(point_list)
        return point_list
    
    def point_to_arrays(self, point_array):
        return_array_x = []
        return_array_y = []
        for element in point_array: 
            #print(point_array)
            return_array_x.append(element.x)
            return_array_y.append(element.y)
            print(element.x, element.y)
       

        return return_array_x, return_array_y





    def distance_between_two_points(self, p1, p2):
            #print("POINTS: ", p1, p2)
            x_len = p1.x - p2.x
            y_len = p1.y - p2.y
            
            hyp = math.sqrt( pow(x_len,2) +  pow(y_len,2)) 
            #print("distance: ", hyp)
            return hyp
            

        
    
       




 
        

        
            
def main(args=None):
    rclpy.init(args=args)
    nine_points_runner = nine_points_algo_v1()
    rclpy.spin(nine_points_runner)
    rclpy.shutdown()
 






if __name__ == '__main__':
    main()