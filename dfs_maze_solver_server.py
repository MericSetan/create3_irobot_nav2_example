#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from slam_sim_interface.srv import DfsMazeSolver
import cv2
import numpy as np

""" 
int32[] maze
float64[] maze_w_coordinates
float64 initial_pose_x
float64 initial_pose_y
---

float64[] nav2_path
"""
class DfsMazeSolverNode(Node):
    def __init__(self):
        super().__init__("dfs_maze_solver")
        self.get_logger().info("dfs_maze_solver_serivisi_baslatildi")
        self.server = self.create_service(DfsMazeSolver,"dfs_maze_solver",self.callback)

    def callback(self,request,response):
        #get request
        maze = request.maze
        maze_w_coordinates = request.maze_w_coordinates
        initial_pose_x = request.initial_pose_x
        initial_pose_y = request.initial_pose_y

        self.gezilen_hucre = []
        self.dfs(maze,initial_pose_x,initial_pose_y)
        waypoints = self.find_waypoints(self.gezilen_hucre,maze_w_coordinates)
        return response
    """ def find_waypoints(gezilen_hucre,maze_w_coordinates):
        
        return waypoints """
    def is_valid_move(self,maze,x, y):
        if x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0]):
            return False
        return maze[x][y] == 0

    def dfs(self,maze,x, y):
        if not self.is_valid_move(maze,x, y):
            return
        
        maze[x][y] = -1  
        self.gezilen_hucre.append((x,y))
        dx = [0, 0, -1, 1]
        dy = [-1, 1, 0, 0]
        for i in range(4):
            new_x = x + dx[i]
            new_y = y + dy[i]
            self.dfs(maze,new_x, new_y)
    


def main(args = None):
    rclpy.init(args=args)
    node = DfsMazeSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()