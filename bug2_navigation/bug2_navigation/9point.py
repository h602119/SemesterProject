import rclpy
from rclpy.node import Node
from bug2_interfaces.srv import NinePoint
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from threading import Thread


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

        #Vales for randomly generating points.
        self.previous_values = []
        self.rejected_values = []

    def nine_point_service(self, request, response):
        if self.is_flooded:
            
            print('Request from controller received')
            self.points_x = []
            self.points_y = []
            
            points = self.gen_9_points()

            for point in points:
                new_point = self.fromArray_ToReal(point.x, point.y)
                self.points_x.append(new_point[0])
                self.points_y.append(new_point[1])

            if request.create_point:
                response.data_x = self.points_x
                response.data_y = self.points_y
                print('Sending 9 points.')
            
            # Run plot_wall_data in a separate thread, so that it is non blocking
            plot_thread = Thread(target=self.plot_wall_data, args=(self.matrix,))
            plot_thread.start()

        return response


    #MAP Callback
    def map_updates_callback(self, msg):
        # Get the info about the walls
        print('Map received')
        self.wall_info = msg.info

        self.wall_data = msg.data

        if not self.is_flooded:
            #Creating a numpy matrix array of the data received from the map.
            self.matrix = np.array(msg.data)
            self.matrix = self.matrix.reshape(self.wall_info.height, self.wall_info.width)

            #Saving a copy of the matrix as map
            self.map = self.matrix

            print('Flooding map')
            self.flood()
            print('Flooding complete...')
        

    #Plot the map data
    def plot_wall_data(self, map_matrix):
        
        
        print('generating 9 points')
        points = self.gen_9_points()
        print('     points generated.')

        #Set to True if you want all points to display.
        show_all_points = True
        if show_all_points:        
            for p in self.rejected_values:
                c = 'red'
                m = 'x'
                plt.scatter(p.x, p.y, color=c ,marker=m, s=10)

            for p in self.previous_values:
                c = 'blue'
                m = '.'
                plt.scatter(p.x, p.y, color=c ,marker=m, s=10)    

        #The 9 new points, will always show
        for p in points:
            c = 'lime'
            m = '.'
            if self.is_outside_house(p.x, p.y) or self.is_wall(p.x, p.y):
                c = 'red'
                m = 'x'
            
            plt.scatter(p.x, p.y, color=c ,marker=m, s=20)


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
    def is_outside_house(self, x, y):    
        return self.matrix[int(y)][int(x)] == -1


    #Returns True if a point is inside a wall
    def is_wall(self, x, y):
        return self.map[int(y)][int(x)] == 100
      
    #Generate 9 points
    def gen_9_points(self):
        
        points = []
        #Select amount of points to generate
        amount = 18

        while(len(points) < amount):
            point = self.generate_random_point()

            if self.is_valid_point(point):
                self.previous_values.append(point)
                points.append(point)
            else:
                self.rejected_values.append(point)
        
        return points



    def generate_random_point(self):
        len = self.wall_info.height  # Values for randomly generating points.
        
        while True:
            # Creating a random number from 0 to length of the map
            rand_x = np.random.uniform(0, len)
            rand_y = np.random.uniform(0, len)

            random_point = Point()
            random_point.x = rand_x
            random_point.y = rand_y

            # Check if the new point is at least 0.25 units away from previous points
            if all(self.distance(random_point, p) >= 75 for p in self.previous_values + self.rejected_values):
                return random_point

    def distance(self, point1, point2):
        return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    
    def is_valid_point(self, p):
        if self.is_wall(p.x, p.y) or self.is_outside_house(p.x, p.y):
                return False
        else:
                return True


def main(args=None):

    rclpy.init(args=args)

    controller = Point_9()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
