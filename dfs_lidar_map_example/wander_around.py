#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.action import Undock,Dock
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from generate_maze_module import GenerateMap
from dfs_maze_solver_module import DfsMazeSolver
import math
import random
#dock islemleri
class DockingClientNode(Node):
    def __init__(self):
        super().__init__('dock_w_nav')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

    def call_undock_action(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()    

        self._send_goal_future = self.undock_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.undock_goal_response_callback)

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.undock_get_result_callback)

    def undock_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.is_docked))
        if not result.is_docked:
            print("Aksiyon basarili: Robot dock cikti.")
        else:
            print("Aksiyon basarili değil: Robot dock edilmedi.")

    def call_dock_action(self):
        goal_msg = Dock.Goal()
        self.dock_action_client.wait_for_server()    

        self.dock_send_goal_future = self.dock_action_client.send_goal_async(goal_msg)

        self.dock_send_goal_future.add_done_callback(self.dock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.dock_get_result_future = goal_handle.get_result_async()
        self.dock_get_result_future.add_done_callback(self.dock_get_result_callback)

    def dock_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.is_docked))
        if  result.is_docked:
            print("Aksiyon basarili: Robot dock edildi.")
        else:
            print("Aksiyon basarili değil: Robot dock edilmedi.")


def flat_to_tuple(flat_data):
    tuple_length = 2  
    original_list = []

    for i in range(0, len(flat_data), tuple_length):
        tuple_item = tuple(flat_data[i:i+tuple_length])
        original_list.append(tuple_item)
    return original_list

def convert_to_square_matrix(input_list):
    list_length = len(input_list)
    side_length = int(list_length ** 0.5)  
    square_matrix = []

    for i in range(0, list_length, side_length):
        square_matrix.append(input_list[i:i+side_length])

    return square_matrix
def get_matrix_properties(matrix):
    num_rows = len(matrix)
    num_columns = len(matrix[0]) if num_rows > 0 else 0
    side_length = num_rows if num_rows == num_columns else None
    
    return num_rows, num_columns, side_length

def create_pose_stamped(navigator:BasicNavigator, position_x,position_y,orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0,0.0,orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def find_initial_pose(maze,maze_w_coordinates,dock_pose_x,dock_pose_y):
    min_distance = float("inf")  # Sonsuz büyük bir değer ile başlatılır
    nearest_index = None

    for row_index, row in enumerate(maze_w_coordinates):
        for col_index, (x, y) in enumerate(row):
            distance = math.sqrt((dock_pose_x - x) ** 2 + (dock_pose_y - y) ** 2)
            if distance < min_distance:
                min_distance = distance
                nearest_index = (row_index, col_index)

    return nearest_index

def main(args=None):
    rclpy.init(args=args)
    dock = DockingClientNode()
    #dock.call_dock_action() # docka sokmak icin
    #dock.call_undock_action() # docktan cikmak icin

    # start nav2
    nav = BasicNavigator()
    
    # --- set initial pose 
    initial_pose = create_pose_stamped(nav,9.077 ,8.077 ,0)
    nav.setInitialPose(initial_pose=initial_pose)

    #--- wait for nav2
    nav.waitUntilNav2Active()    
      
    # map to maze
    # parametreler ile yap
    generate_map_obj = GenerateMap()

    image_path = '/home/mrc_stn/ros2_ws/src/slam_sim/slam_sim/map_test.png'
    diviation = 6.1 # metre
    map_resolution = 0.05 # metre

    maze,maze_w_coordinates = generate_map_obj.callback(image_path,diviation,map_resolution)
    
    for row in maze:
        print(row)

    for row in maze_w_coordinates:
        print(row)

    #dfs maze solver 
    dock_pose_x = 8.99 
    dock_pose_y = 8.0
    initial_pose_x,initial_pose_y = find_initial_pose(maze,maze_w_coordinates,dock_pose_x,dock_pose_y)
    print("initial_pose_maze",maze_w_coordinates[initial_pose_x][initial_pose_y])

    maze_solver_obj = DfsMazeSolver()
    #print(initial_pose_x,initial_pose_y )
    path = maze_solver_obj.callback(maze,maze_w_coordinates,initial_pose_x,initial_pose_y)
    
    waypoints = []
     
    for i,a in enumerate(path):
        waypoints.append(create_pose_stamped(nav,float(a[0]) ,float(a[1]) ,random.uniform(0, 3.12)))
    print(path)
    
    """ dock.call_undock_action() #baslangicta docktan cikiyor
    for i,w in enumerate(waypoints):
        nav.goToPose(w)
        while not nav.isTaskComplete():
            result = nav.getResult()
            feedback = nav.getFeedback()
            if result == 3:
                print("basarisiz")
                print(i)
            elif result == 1:
                print("basarili")
                print(i)
            #print(result)
            
    dock.call_dock_action()    # docka giriyor     """
      
    # ! mazedeki sıfırlar ve path uzunlugunu mutlaka kontrol et
    rclpy.spin(dock)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
