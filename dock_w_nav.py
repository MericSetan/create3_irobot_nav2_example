#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.action import Undock,Dock
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

"""
# Request
---
# Result
bool is_docked
---
# Feedback
"""

#Dock action client
class UndockClientNode(Node):
    def __init__(self):
        super().__init__('dock_w_nav')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')
    # undocking
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
            print("Aksiyon basarili değil: Robot dock cikamadi.")

    #docking
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
            print("Aksiyon basarili değil: Robot dock edilemedi.")

#nav2
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


def main(args=None):
    rclpy.init(args=args)
    dock = UndockClientNode()
    nav = BasicNavigator()

    # --- set initial pose 
    initial_pose = create_pose_stamped(nav,9.077 ,8.077 ,0.0)
    nav.setInitialPose(initial_pose=initial_pose)
    #--- wait for nav2
    nav.waitUntilNav2Active()
    #--- describe waypoints
    goal_pose1 = create_pose_stamped(nav, 4.076 ,7.999 ,-1.362)
    goal_pose2 = create_pose_stamped(nav,  9.742 ,2.029, 3.119) #bu asamada
    goal_pose3 = create_pose_stamped(nav,  8.68 ,8.0 ,0.0) # baslangıc konumuna yakin bir yere geri donduruyoruz 
    waypoints = [goal_pose1, goal_pose2, goal_pose3]


    dock.call_undock_action() #baslangicta docktan cikiyor
    nav.followWaypoints(waypoints) # sırayla noktalara gidiyor
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
    print(nav.getResult())
    dock.call_dock_action()    # docka giriyor  


    rclpy.spin(dock)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
