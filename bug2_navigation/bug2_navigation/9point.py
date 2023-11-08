import rclpy
from rclpy.node import Node
from bug2_interfaces.srv import NinePoint
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point 
import math  
import random
import rclpy

from std_msgs.msg import Header 
from nav_msgs.msg import MapMetaData

import time
from collections import deque



class Point_9(Node):
    def __init__(self):
        super().__init__("Nine_Point")
        self.get_logger().info("9Point algorithm started")

        self.srv = self.create_service(NinePoint, 'nine_point', self.new_nine_point_service)
       
        # Subscribe to the /map topic
        map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.past_points = []
        self.new_path = []
        self.x, self.y = [],[]
        self.x_grid, self.y_grid = [],[]
        self.two_d_array = []
        self.wall_info = MapMetaData()
        self.occupancy_grid_data = OccupancyGrid()
        self.two_d_array = [[]]
        

    def new_nine_point_service(self, request, response):

        print('Request from controller received')
  
        while self.wall_info.resolution == 0: 
            time.sleep(5)

        self.new_path = self.generate_points()
        self.x, self.y = self.point_to_arrays(self.new_path)

        if(request.create_point):
            response.data_x = self.x
            response.data_y = self.y
            print('Sending points.')

        print(f" X values: {response.data_x} \n Y values: {response.data_y}")
        self.visualize_map(self.occupancy_grid_data)
        return response
    


    def map_callback(self, msg):

        self.wall_info = msg.info
        self.occupancy_grid_data = msg
        #print(msg.data)        
        self.two_d_array = np.array(msg.data).reshape(( msg.info.width, msg.info.height))
        self.two_d_array = self.flood(self.two_d_array)
        self.two_d_array = self.two_d_array[::-1, :]
        self.two_d_array = np.rot90(self.two_d_array, k=3)


       

        


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
    
    def flood(self, input_matrix):
        # Mark all accessible areas from a starting point within the walls
        # with a special value, e.g., -1
        start_x, start_y = 0,0  # You need to implement this
        queue = deque([(start_x, start_y)])
        visited = np.zeros_like(input_matrix, dtype=bool)
        visited[start_y][start_x] = True
        input_matrix[start_y][start_x] = -1

        while queue:
            x, y = queue.popleft()

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.wall_info.width and 0 <= ny < self.wall_info.height:
                    if input_matrix[ny][nx] == 0 and not visited[ny][nx]:
                        visited[ny][nx] = True
                        input_matrix[ny][nx] = 100
                        queue.append((nx, ny))
        return input_matrix
    
    
    def is_in_wall(self, x,y):
        x = int(x)
        y = int(y)
        #print(self.two_d_array[x][y])
        if(self.two_d_array[x][y] > 50):
            return True
        return False
    

    def real_world_to_grid_coords(self, x_real, y_real):

        map_info = self.wall_info
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        resolution = map_info.resolution
        width = map_info.width
        height = map_info.height

        # Calculate grid indices
        new_x = (x_real - origin_x) / resolution
        new_y = (y_real - origin_y) / resolution

        # Ensure the indices are within bounds
        new_x = max(0, min(new_x, width - 1))
        new_y = max(0, min(new_y, height - 1))

        
        #print(new_x, new_y)
        

        return new_x, new_y
    def visualize_map(self, map_msg):
        x_list = []
        y_list = []
        print(self.x, self.y)

        for index in range(len(self.x)):
            print(self.x[index], self.y[index])
            tmp_x, tmp_y = self.real_world_to_grid_coords(self.x[index], self.y[index])
            x_list.append(tmp_x)
            y_list.append(tmp_y)

        self.x_grid, self.y_grid = x_list,y_list
        occupied_cells = self.two_d_array >= 50  
        free_cells = self.two_d_array < 50
        fig, ax = plt.subplots()
        #ax.imshow(occupied_cells, cmap='gray', interpolation='none', extent=[0, map_msg.info.width, 0, map_msg.info.height])
        print_map = np.rot90(self.two_d_array, k=1)
        ax.imshow(print_map, cmap='binary', interpolation='none', extent=[0, map_msg.info.width, 0, map_msg.info.height])

        # Set labels and title
        ax.set_xlabel('X (Cells)')
        ax.set_ylabel('Y (Cells)')
        ax.set_title('Occupancy Grid')
        print(x_list, y_list)
        plt.scatter(x_list, y_list, color='green', marker='o', label='Scatter Points', s=[5,5])


        plt.show()

    
    def generate_points(self):
        
        point_list = []
        width = 20 
        number_of_points = 9
        randomness = 2.0
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
                #print("random float1: ",random_float)
                tmp_point.x = point_x*distance_pr_point + random_float + formula

                random_float = random.uniform(-half_rand, half_rand)
                #print("random float2: ",random_float)
                tmp_point.y = (point_y*distance_pr_point + random_float + formula)*flip

                #pick a new point untill its far enough away from other points
                spawn_attempts = 0
                grid_x, grid_y = self.real_world_to_grid_coords(tmp_point.x,tmp_point.y)
                while any(self.distance_between_two_points(ele,tmp_point) < too_close for ele in self.past_points) or self.is_in_wall(grid_x,grid_y) and spawn_attempts < 10: # add check to see if element is on wall 

                    
                    random_float = random.uniform(-half_rand, half_rand)
                    #print("random float1: ",random_float)
                    tmp_point.x = point_x*distance_pr_point + random_float + formula

                    random_float = random.uniform(-half_rand, half_rand) 
                    #print("random float2: ",random_float)
                    tmp_point.y = (point_y*distance_pr_point + random_float + formula)*flip

                    #print(tmp_point.x, tmp_point.y)
                    grid_x, grid_y = self.real_world_to_grid_coords(tmp_point.x,tmp_point.y)
                    spawn_attempts += 1
                
                #print("new point beeing added: ",tmp_point)
                if spawn_attempts < 10:
                    point_list.append(tmp_point)
                    self.past_points.append(tmp_point)
        #print(len(point_list))
        #print(point_list)
        return point_list

  

    


    
    
    



      


      

def main(args=None):

    rclpy.init(args=args)

    controller = Point_9()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
