import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt

class Map(Node):
    def __init__(self):
        super().__init__('wall_printer')

        # Create a subscription to the `/map` topic
        self.map_updates_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_updates_callback, 10
        )

        self.is_flooded = False

        #A save of the original numpy map.
        self.map = None

        #Numpy map for further use 
        self.matrix = []

        self.wall_data = 0
        self.wall_info = 0

    def map_updates_callback(self, msg):
        # Get the info about the walls

        self.wall_info = msg.info

        print(f'Wall width {self.wall_info.width}')

        print(f'Wall height {self.wall_info.height}')

        print(f'Wall resolution {self.wall_info.resolution}')
        

        # Get the data about the walls
        
        print('finding Walls... ')

        self.wall_data = msg.data

        #Creating a numpy matrix array of the data received from the map.
        self.matrix = np.array(msg.data)
        self.matrix = self.matrix.reshape(self.wall_info.height, self.wall_info.width)
        
        #Rotating the map 3*90deg since [0,0] in np array is bottom left, but the original has [0,0] bottom right.
        self.matrix = np.rot90(self.matrix, k=3)

        #Saving a copy of the matrix as map
        self.map = self.matrix

        self.plot_wall_data(self.matrix)
    
    #Plot the wall data
    def plot_wall_data(self, map_matrix):
        
        #Flood map if this has not been done.
        if not self.is_flooded:
            self.flood()

        #For testing purposes
        """
        check_x = [355,707,1026,1665,1464,1524,664,220,268,301,896,1719,1719,187,983,994,1313,691]
        check_y = [1807,1623,1575,1678,1104,1044,774,1255,1169,1694,1553,1678,509,482,108,1569,1088,833]

        for i in range(len(check_x)):

            self.is_wall(check_x[i], check_y[i])
            
            if self.is_inside_house(check_x[i], check_y[i]) >= 0:
                print(f'{check_x[i]}, {check_y[i]} is within the outer walls')
            else:
                print(f'{check_x[i]}, {check_y[i]} is outside the outer walls')
        """
            #Test was successful
        real_points_x = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]
        real_points_y = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]

        for i in range(len(real_points_x)):
            points = self.fromReal_ToArray(real_points_x[i], real_points_y[i])
            c = 'lime'
            m = '.'
            if self.is_inside_house(points[0], points[1]) or self.is_wall(points[0], points[1]):
                c = 'red'
                m = 'x'
            
            plt.scatter(points[0], points[1], color=c ,marker=m, s=20)

        plt.imshow(map_matrix, cmap='gray', origin='lower')
        plt.show()    

    #Convert a real world point to an array point ( Real : from [-10,-10] to [10,10] )
    def fromReal_ToArray(self, x, y):
        if (x > 10) or (x < -10) or (y > 10) or (y < -10):
            return [0,0]
        x = (x+10)*100
        y = (y+10)*100
        x -= 1
        y -= 1
        return [x,y]
    
    #Convert an array point to a real world point ( Array : from [0,0] to [1999, 1999] )
    def fromArray_ToReal(self, x, y):
        if x > 2000 or y > 2000 or x < 0 or y < 0:
            return [0,0]
        x = (x/100)-10
        y = (y/100)-10

        return [x,y]
    
    #Flooding method...
    def flood(self):
        self.is_flooded = True
        # Assume 0 is open space, 100 (or any non-zero value) is a wall
        # Use a queue to perform BFS
        from collections import deque
        queue = deque([(0, 0)])  # starting point, should be inside the wall
        while queue:
            x, y = queue.popleft()
            if self.matrix[y][x] == 0:  # If the space is open
                self.matrix[y][x] = -1  # Mark as visited
                # Enqueue the neighboring points
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.wall_info.width and 0 <= ny < self.wall_info.height:
                        queue.append((nx, ny))
    
    #Check method check if a given point is within our outside the outer walls.   
    def is_inside_house(self, x, y):
        
        return self.matrix[y][x] == -1

    #Returns True if a point is inside a wall
    def is_wall(self, x, y):
        
        return self.map[y][x] == 100
            
            

def main(args=None):
    rclpy.init(args=args)
    wall_printer = Map()
    rclpy.spin(wall_printer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
