#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

class RealSenseViewer(Node):
    def __init__(self):
        super().__init__('realsense_viewer')
        
        self.bridge = CvBridge()
        
        # Image storage
        self.rgb_image = None
        self.depth_image = None
        self.infra1_image = None
        self.infra2_image = None
        
        # Flags
        self.show_depth = True
        self.show_infra = False
        self.depth_colormap = cv2.COLORMAP_JET
        
        # Subscribe to RealSense topics
        # Adjust these topic names based on your RealSense configuration
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        self.infra1_sub = self.create_subscription(
            Image,
            '/camera/camera/infra1/image_rect_raw',
            self.infra1_callback,
            10)
        
        self.infra2_sub = self.create_subscription(
            Image,
            '/camera/camera/infra2/image_rect_raw',
            self.infra2_callback,
            10)
        
        self.get_logger().info('RealSense Viewer started!')
        self.get_logger().info('Subscribing to topics:')
        self.get_logger().info('  - /camera/camera/color/image_raw (RGB)')
        self.get_logger().info('  - /camera/camera/depth/image_rect_raw (Depth)')
        self.get_logger().info('  - /camera/camera/infra1/image_rect_raw (Infrared 1)')
        self.get_logger().info('  - /camera/camera/infra2/image_rect_raw (Infrared 2)')
        self.get_logger().info('')
        self.get_logger().info('Controls:')
        self.get_logger().info('  d - Toggle depth view')
        self.get_logger().info('  i - Toggle infrared view')
        self.get_logger().info('  c - Cycle depth colormap (Jet/Hot/Cool/Rainbow)')
        self.get_logger().info('  q or ESC - Quit')
        self.get_logger().info('')
        
        # Create display window
        cv2.namedWindow('RealSense Viewer', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RealSense Viewer', 1280, 720)
        
        # Timer for display update
        self.display_timer = self.create_timer(0.033, self.display_callback)  # ~30 fps
        
        self.frame_count = 0
        self.last_fps_time = self.get_clock().now()
        self.fps = 0.0
        
    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def depth_callback(self, msg):
        try:
            # Depth is usually 16-bit
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def infra1_callback(self, msg):
        try:
            self.infra1_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Infrared 1 conversion error: {e}')
    
    def infra2_callback(self, msg):
        try:
            self.infra2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Infrared 2 conversion error: {e}')
    
    def process_depth_image(self, depth_image):
        """Convert depth image to colorized display"""
        # Normalize depth for visualization
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        # Apply colormap
        depth_colorized = cv2.applyColorMap(depth_normalized, self.depth_colormap)
        
        # Add depth value at center
        h, w = depth_image.shape[:2]
        center_x, center_y = w // 2, h // 2
        center_depth = depth_image[center_y, center_x]
        
        # Draw crosshair and depth value
        cv2.line(depth_colorized, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(depth_colorized, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
        
        # Display depth value in meters
        depth_m = center_depth / 1000.0 if center_depth > 0 else 0
        cv2.putText(depth_colorized, f'Center: {depth_m:.2f}m', 
                   (center_x + 30, center_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return depth_colorized
    
    def display_callback(self):
        """Update the display window"""
        # Calculate FPS
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.last_fps_time).nanoseconds / 1e9
            self.fps = 30 / elapsed if elapsed > 0 else 0
            self.last_fps_time = current_time
        
        # Create display canvas
        if self.rgb_image is not None:
            display_images = []
            
            # RGB image
            rgb_display = self.rgb_image.copy()
            
            # Add FPS counter
            cv2.putText(rgb_display, f'FPS: {self.fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add labels
            cv2.putText(rgb_display, 'RGB Camera', (10, rgb_display.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            display_images.append(rgb_display)
            
            # Depth image (if enabled and available)
            if self.show_depth and self.depth_image is not None:
                depth_display = self.process_depth_image(self.depth_image)
                
                # Resize to match RGB height
                h_rgb = rgb_display.shape[0]
                scale = h_rgb / depth_display.shape[0]
                w_depth = int(depth_display.shape[1] * scale)
                depth_display = cv2.resize(depth_display, (w_depth, h_rgb))
                
                cv2.putText(depth_display, 'Depth', (10, h_rgb - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                display_images.append(depth_display)
            
            # Infrared images (if enabled and available)
            if self.show_infra:
                if self.infra1_image is not None:
                    infra1_display = cv2.cvtColor(self.infra1_image, cv2.COLOR_GRAY2BGR)
                    h_rgb = rgb_display.shape[0]
                    scale = h_rgb / infra1_display.shape[0]
                    w_infra = int(infra1_display.shape[1] * scale)
                    infra1_display = cv2.resize(infra1_display, (w_infra, h_rgb))
                    
                    cv2.putText(infra1_display, 'Infrared 1', (10, h_rgb - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    display_images.append(infra1_display)
                
                if self.infra2_image is not None:
                    infra2_display = cv2.cvtColor(self.infra2_image, cv2.COLOR_GRAY2BGR)
                    h_rgb = rgb_display.shape[0]
                    scale = h_rgb / infra2_display.shape[0]
                    w_infra = int(infra2_display.shape[1] * scale)
                    infra2_display = cv2.resize(infra2_display, (w_infra, h_rgb))
                    
                    cv2.putText(infra2_display, 'Infrared 2', (10, h_rgb - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    display_images.append(infra2_display)
            
            # Combine images horizontally
            combined = np.hstack(display_images)
            
            # Show the combined image
            cv2.imshow('RealSense Viewer', combined)
        else:
            # Show waiting screen
            waiting_screen = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(waiting_screen, 'Waiting for camera data...', (100, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(waiting_screen, 'Make sure RealSense is running!', (100, 280),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
            cv2.imshow('RealSense Viewer', waiting_screen)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:  # q or ESC
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()
        elif key == ord('d'):
            self.show_depth = not self.show_depth
            self.get_logger().info(f'Depth view: {"ON" if self.show_depth else "OFF"}')
        elif key == ord('i'):
            self.show_infra = not self.show_infra
            self.get_logger().info(f'Infrared view: {"ON" if self.show_infra else "OFF"}')
        elif key == ord('c'):
            # Cycle through colormaps
            colormaps = [cv2.COLORMAP_JET, cv2.COLORMAP_HOT, cv2.COLORMAP_COOL, 
                        cv2.COLORMAP_RAINBOW, cv2.COLORMAP_VIRIDIS]
            colormap_names = ['Jet', 'Hot', 'Cool', 'Rainbow', 'Viridis']
            
            current_idx = colormaps.index(self.depth_colormap)
            next_idx = (current_idx + 1) % len(colormaps)
            self.depth_colormap = colormaps[next_idx]
            self.get_logger().info(f'Depth colormap: {colormap_names[next_idx]}')


def main(args=None):
    # Check if cv_bridge is available
    try:
        from cv_bridge import CvBridge
    except ImportError:
        print('')
        print('ERROR: cv_bridge not found!')
        print('Install it with:')
        print('  sudo apt install ros-humble-cv-bridge python3-opencv')
        print('')
        return 1
    
    rclpy.init(args=args)
    
    viewer = RealSenseViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

