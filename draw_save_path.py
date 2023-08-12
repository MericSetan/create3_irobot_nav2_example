#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException
import csv 
from datetime import datetime
from rcl_interfaces.msg import ParameterDescriptor
import cv2 

# ! lidar haritası icin pgm to png islemi gereklidir 
# ! yaml dosyası icerisinde duzenleme gerekebilir

image_path = "/home/mrc_stn/ros2_ws/src/slam_sim/slam_sim/map_test2.png"
file_name = "path.csv"

class DrawSavePath(Node):
    def __init__(self):
        super().__init__('draw_save_path')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.counter = self.create_timer(3,self.get_tf_data)

    def get_tf_data(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            #x,y,z verileri
            x = trans.transform.translation.x 
            y = trans.transform.translation.y
            z = trans.transform.rotation.z

            # zaman verisi (sunucu saati), ! mesaj uzerinden alinabilir
            now = datetime.now()
            time = now.strftime("%H:%M:%S")
            """
            # terminal
            self.get_logger().info("time : "+str(time))
            self.get_logger().info("x : "+str(x))
            self.get_logger().info("y : "+str(y))
            self.get_logger().info("z : "+str(z))"""

            p_x,p_y = self.meter_to_pixel(x,y,0.05)
            self.get_logger().info("x : "+str(x)+ " pixel"+str(p_x))
            self.get_logger().info("y : "+str(y)+" pixel"+str(p_y))
            
            #save as csv
            self.save_tf_data(time ,p_x,p_y,z)
            #draw img
            self.draw_image(x,y)

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

    def save_tf_data(self,time,x=0.0,y=0.0,z=0.0):
        with open(file_name, mode='a',newline='') as csv_file:
            fieldnames = ['time', 'x', 'y','z']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            if csv_file.tell() == 0:
                writer.writeheader()
            writer.writerow({'time': time, 'x': x, 'y': y, 'z': z})

    def draw_image(self,x,y):
        p_x,p_y = self.meter_to_pixel(x,y,0.05)
        image = cv2.imread(image_path) 
        cv2.circle(image, (p_x, p_y), 1, (0,255,0), -1)
        cv2.imwrite(image_path,image)

    def meter_to_pixel(self,meter_x, meter_y, resolution):
        pixel_x = int(meter_x / resolution)
        pixel_y = int(meter_y / resolution)
        return pixel_x, pixel_y        

def main(args=None):
    rclpy.init(args=args)
    draw_save_path = DrawSavePath()
    rclpy.spin(draw_save_path)
    draw_save_path.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
