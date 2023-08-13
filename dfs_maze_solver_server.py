#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from slam_sim_interface.srv import DfsMazeSolver

""" 
int32[] maze
float64[] maze_w_coordinates
float64 initial_pose_x
float64 initial_pose_y
---

float64[] nav2_path
"""
gezilen_hucre = []
class DfsMazeSolverNode(Node):
    def __init__(self):
        super().__init__("dfs_maze_solver")
        self.get_logger().info("dfs_maze_solver_serivisi_baslatildi")
        self.server = self.create_service(DfsMazeSolver,"dfs_maze_solver_server",self.callback)

    def callback(self,request,response):
        #get request
        maze = request.maze
        maze_w_coordinates = request.maze_w_coordinates
        initial_pose_x = request.initial_pose_x
        initial_pose_y = request.initial_pose_y

        self.dfs(maze,0, 0)
        # dizi boyutları eşit mi kontrolü yaabilirsin
        print(gezilen_hucre)

        selected_values = []

        for index in gezilen_hucre:
            row, col = index
            value = maze_w_coordinates[row][col]
            selected_values.append(value)
        print(selected_values)

        if selected_values ==[]:
            print("initial pose yanlis secildi")
        response.nav2_path = selected_values
        return response    
    def is_valid_move(self,maze,x, y):
        if x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0]):
            return False
        return maze[x][y] == 0

    def dfs(self,maze,x, y):
        if not self.is_valid_move(maze,x, y):
            return
        
        maze[x][y] = -1  # Ziyaret edildi olarak işaretle
        #print(f"Ziyaret edilen hücre: ({x}, {y})")
        gezilen_hucre.append((x,y))
        
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