#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node

""" 
int32[] maze
float64[] maze_w_coordinates
float64 initial_pose_x
float64 initial_pose_y
---

float64[] nav2_path
"""
gezilen_hucre = []

class DfsMazeSolver():
    def __init__(self):
        print("maze cozuluyor")

    def callback(self,maze,maze_w_coordinates,initial_pose_x,initial_pose_y):
        
        self.dfs(maze,initial_pose_x, initial_pose_y)
        # dizi boyutları eşit mi kontrolü yaabilirsin
        
        selected_values = []

        for index in gezilen_hucre:
            row, col = index
            value = maze_w_coordinates[row][col]
            selected_values.append(value)
        #print(selected_values)

        if selected_values ==[]:
            print("initial pose yanlis secildi")
        
        return selected_values    
    
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
            


