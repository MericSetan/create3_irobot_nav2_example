#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from slam_sim_interface.srv import MapConverter
import cv2
import numpy as np
""" 
string image_path
int32 diviation
float32 map_resulation
---
int32[] maze
float64[] maze_w_coordinates 
"""

""" tested with
    image_path = '/home/mrc_stn/ros2_ws/src/slam_sim/slam_sim/map_test.png'
    diviation = 2 # metre
    map_resulation = 0.05 # metre 
"""
class GenerateMap(Node):
    def __init__(self):
        super().__init__("generate_map_server")
        self.get_logger().info("map_serivisi_baslatildi")
        self.server = self.create_service(MapConverter,"generate_map_server",self.callback)

    def callback(self,request,response):
        #get request
        image_path = request.image_path
        diviation = request.diviation
        map_resulation = request.map_resulation

        image = cv2.imread(image_path)

        grid_size = self.meters_to_pixels(diviation, map_resulation)
        # eger bolmede hata varsa acilabilir
        #padded_image = pad_image(image, grid_size)

        grids = self.div_map(image, grid_size)
        grid_centers = self.find_original_grid_centers(image,grids,grid_size)
        grid_centers_meter = []
        for i in grid_centers:
            grid_centers_meter.append(self.pixels_to_meters(i,map_resulation))


        #find clear grid to go to the goal_pose
        threshold = 240  
        white_presence_array = self.create_white_presence_array(grids, threshold)

        response.maze = white_presence_array
        flat_data = [item for sublist in grid_centers_meter for item in sublist]
        """
        tuple_length = 2  
        original_list = []

        for i in range(0, len(flat_data), tuple_length):
            tuple_item = tuple(flat_data[i:i+tuple_length])
            original_list.append(tuple_item)

        """
        response.maze_w_coordinates = flat_data

        return response
    
    
    def div_map(self,image, grid_size):
        
        num_rows = image.shape[0] // grid_size[0]
        num_cols = image.shape[1] // grid_size[1]

        grids = []

        for row in range(num_rows):
            for col in range(num_cols):
                grid = image[row * grid_size[0]: (row + 1) * grid_size[0],
                            col * grid_size[1]: (col + 1) * grid_size[1]]
                grids.append(grid)

        return grids

    def meters_to_pixels(self,meters, map_resulation):
        #only square
        return (int(meters * 100 * map_resulation),int(meters * 100 * map_resulation))

    def pixels_to_meters(self,pixels, map_resulation):
        return (pixels[0] / (100 * map_resulation), pixels[1] / (100 * map_resulation))

    def check_middle_pixel_color(self,grid, threshold):
        height, width, _ = grid.shape
        middle_pixel = grid[height // 2, width // 2]
        rtrn_bool = np.all(middle_pixel > threshold)
        return rtrn_bool

    def create_white_presence_array(self,grids, threshold):
        presence_array = []
        for grid in grids:
            is_above_threshold = self.check_middle_pixel_color(grid, threshold)
            if is_above_threshold:
                presence_array.append(1)  
            else:
                presence_array.append(0)  
        return presence_array

    def find_original_grid_centers(self,image,grids,grid_size):
        grid_centers = []  

        for i, grid in enumerate(grids):
            # Orta noktanın koordinatlarını hesapla
            center_x = (i % (image.shape[1] // grid_size[1]) + 0.5) * grid_size[1]
            center_y = (i // (image.shape[1] // grid_size[1]) + 0.5) * grid_size[0]
            grid_centers.append((center_x, center_y))
        return grid_centers

    


def main(args = None):
    rclpy.init(args=args)
    node = GenerateMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()