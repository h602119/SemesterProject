import rclpy
from rclpy.node import Node
from bug2_interfaces.srv import NinePoint
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt


class Point_9(Node):
    def __init__(self):
        super().__init__("Nine_Point")
        self.get_logger().info("9Point algorithm started")

        self.srv = self.create_service(NinePoint, 'nine_point', self.nine_point_service)

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

        self.points_x = []
        self.points_y = []

    #9 POINT Callback
    def nine_point_service(self, request, response):
        
        print('Request from controller received')
        points = self.generate9Points()

        self.points_x.append(points[0])
        self.points_y.append(points[1])   
                
        
        if(request.create_point):
            response.data_x = points[0]
            
            response.data_y = points[1]
            print('Sending 9 points.')
        return response

    #The algorithm that generates 9 points.
    def generate9Points(self):
        p = [   self.fromArray_ToReal(1199, 904),
                self.fromArray_ToReal(1383, 628),
                self.fromArray_ToReal(1605, 877),
                self.fromArray_ToReal(1735, 828),
                self.fromArray_ToReal(1638, 417),
                self.fromArray_ToReal(923, 417),
                self.fromArray_ToReal(923, 590),
                self.fromArray_ToReal(652, 590),
                self.fromArray_ToReal(680, 1407),
                self.fromArray_ToReal(815, 1683), ]

        x = []
        y = []

        for i in range(len(p)):
            x.append(p[i][0])
            y.append(p[i][1])
        
        points = [x, y]

        return points
    

    #MAP Callback
    def map_updates_callback(self, msg):
        # Get the info about the walls
        print('Map received')
        self.wall_info = msg.info

        self.wall_data = msg.data

        #Creating a numpy matrix array of the data received from the map.
        self.matrix = np.array(msg.data)
        self.matrix = self.matrix.reshape(self.wall_info.height, self.wall_info.width)

        #Saving a copy of the matrix as map
        self.map = self.matrix

        self.plot_wall_data(self.matrix)

    #Plot the map data
    def plot_wall_data(self, map_matrix):
        
        #Flood map if this has not been done.
        if not self.is_flooded:
            print('Flooding map')
            self.flood()
            print('completed...')
        
        arr_points = self.generate9Points()
        real_points_x = arr_points[0]
        real_points_y = arr_points[1]

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
        x += 1
        y += 1
        x = (x/100)-10
        y = (y/100)-10

        return [x,y]
    
    #Flooding algorithm
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
        return self.matrix[int(y)][int(x)] == -1

    #Returns True if a point is inside a wall
    def is_wall(self, x, y):
        return self.map[int(y)][int(x)] == 100
      

def main(args=None):

    rclpy.init(args=args)

    controller = Point_9()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
