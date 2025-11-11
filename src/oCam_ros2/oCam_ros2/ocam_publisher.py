#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

class OcamPublisher(Node):
    def __init__(self):
        super().__init__('ocam_publisher')
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Parameters (compatible with camera_params.yaml)
        self.declare_parameter('video_device', '/dev/video4')
        self.camera_device = self.get_parameter('video_device').value
        self.declare_parameter('image_width', 1280)
        self.width = self.get_parameter('image_width').value
        self.declare_parameter('image_height', 720)
        self.height = self.get_parameter('image_height').value
        self.declare_parameter('framerate', 30.0)
        self.frame_rate = self.get_parameter('framerate').value
        self.declare_parameter('frame_id', 'camera4_frame')
        self.frame_id = self.get_parameter('frame_id').value
        self.declare_parameter('camera_info_url', 'file:///home/cyjung/made_in_Korea/src/oCam_ros2/config/camera_info.yaml')
        self.camera_info_path = self.get_parameter('camera_info_url').value.replace('file://', '')

        # Initialize camera
        if isinstance(self.camera_device, str):
            self.cap = cv2.VideoCapture(self.camera_device)
        else:
            self.cap = cv2.VideoCapture(int(self.camera_device))
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        self.cv_bridge = CvBridge()
        
        # Load camera info
        self.camera_info = self.load_camera_info()
        
        # Timer for publishing
        time_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.get_logger().info(f"Camera Device: {self.camera_device}")
        self.get_logger().info(f"Video Width: {self.width}")
        self.get_logger().info(f"Video Height: {self.height}")
        self.get_logger().info(f"Frame Rate: {self.frame_rate}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")

    def load_camera_info(self):
        camera_info = CameraInfo()
        try:
            with open(self.camera_info_path, 'r') as file:
                calib_data = yaml.safe_load(file)
                
            camera_info.header.frame_id = self.frame_id
            camera_info.width = calib_data['image_width']
            camera_info.height = calib_data['image_height']
            camera_info.distortion_model = calib_data['distortion_model']
            
            camera_info.k = calib_data['camera_matrix']['data']
            camera_info.d = calib_data['distortion_coefficients']['data']
            camera_info.r = calib_data['rectification_matrix']['data']
            camera_info.p = calib_data['projection_matrix']['data']
            
        except Exception as e:
            self.get_logger().warn(f"Failed to load camera info: {e}")
            # Set default values
            camera_info.header.frame_id = self.frame_id
            camera_info.width = self.width
            camera_info.height = self.height
            camera_info.distortion_model = 'plumb_bob'
            
        return camera_info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Resize frame if needed
            frame = cv2.resize(frame, (self.width, self.height))
            
            # Convert to ROS message
            timestamp = self.get_clock().now().to_msg()
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = self.frame_id
            
            # Update camera info timestamp
            self.camera_info.header.stamp = timestamp
            
            # Publish messages
            self.image_publisher.publish(img_msg)
            self.camera_info_publisher.publish(self.camera_info)
        else:
            self.get_logger().warn("Failed to capture frame from camera")

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main():
    rclpy.init()
    node = OcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()