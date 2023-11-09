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

        self.detected = False

        self.matrix = []

        self.wall_data = 0
        self.wall_info = 0

        self.outer_walls = set()

    def map_updates_callback(self, msg):
        # Get the info about the walls

        self.wall_info = msg.info

        print(f'Wall width {self.wall_info.width}')

        print(f'Wall height {self.wall_info.height}')

        print(f'Wall resolution {self.wall_info.resolution}')
        

        # Get the data about the walls
        
        print('finding Walls... ')

        self.is_flooded = False

        self.wall_data = msg.data

        self.matrix = np.array(msg.data)
        self.matrix = self.matrix.reshape(self.wall_info.height, self.wall_info.width)
        
        self.matrix = np.rot90(self.matrix, k=3)

        self.plot_wall_data(self.matrix)


    def plot_wall_data(self, map_matrix):
        if not self.detected:
            self.detect_outer_walls()  # Call the flood-fill method

        check_x = [1000, 1329, 485, 301, 1546, 842, 1897, 1697, 382]
        check_y = [900, 1342, 1667, 709, 676, 1905, 1786, 173, 319]
        for i in range(len(check_x)):
            if self.is_inside_outer_walls(check_x[i], check_y[i]):
                print(f"Point ({check_x[i]}, {check_y[i]}) is inside the outer wall.")
            else:
                print(f"Point ({check_x[i]}, {check_y[i]}) is outside the outer wall.")

        points = self.pointToPlot(0,-1)
        plt.scatter(points[0], points[1],color='blue',marker='o',s=20)

        plt.imshow(map_matrix, cmap='gray', origin='lower')
        plt.show()
    

    def pointToPlot(self, x, y):
        if (x > 10) or (x < -10) or (y > 10) or (y < -10):
            return [0,0]
        x = (x+10)*100
        y = (y+10)*100
        return [x,y]
    
    def plotToPoint(self, x, y):
        #TODO
        pass

    def detect_outer_walls(self):
        self.outer_walls.clear()  # Clear previous data
        for y in range(self.matrix.shape[0]):
            for x in range(self.matrix.shape[1]):
                if self.matrix[y, x] == 100:
                    if (
                        x - 1 >= 0 and self.matrix[y, x - 1] == 0
                        or x + 1 < self.matrix.shape[1] and self.matrix[y, x + 1] == 0
                        or y - 1 >= 0 and self.matrix[y - 1, x] == 0
                        or y + 1 < self.matrix.shape[0] and self.matrix[y + 1, x] == 0
                    ):
                        self.outer_walls.add((x, y))
                        print(f"Added ({x}, {y}) as an outer wall")


    def is_inside_outer_walls(self, x, y):
        if ((x, y) in self.outer_walls  or x < 0 or x >= self.matrix.shape[1] or y < 0 or y >= self.matrix.shape[0]):
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    wall_printer = Map()
    rclpy.spin(wall_printer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
