#!/usr/bin/env python3
"""
Stream RealSense camera to web browser
Useful for viewing from any device on the network
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from flask import Flask, Response
import threading

app = Flask(__name__)

class RealSenseWebStreamer(Node):
    def __init__(self):
        super().__init__('realsense_web_streamer')
        
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.lock = threading.Lock()
        
        # Subscribe to RealSense topics
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
        
        self.get_logger().info('RealSense Web Streamer started!')
        self.get_logger().info('Access streams at:')
        self.get_logger().info('  RGB:   http://<jetson-ip>:5000/rgb')
        self.get_logger().info('  Depth: http://<jetson-ip>:5000/depth')
        self.get_logger().info('  Both:  http://<jetson-ip>:5000/')
        
    def rgb_callback(self, msg):
        try:
            with self.lock:
                self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Colorize depth
            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            with self.lock:
                self.depth_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def get_rgb_frame(self):
        with self.lock:
            if self.rgb_image is not None:
                return self.rgb_image.copy()
        return None
    
    def get_depth_frame(self):
        with self.lock:
            if self.depth_image is not None:
                return self.depth_image.copy()
        return None
    
    def get_combined_frame(self):
        rgb = self.get_rgb_frame()
        depth = self.get_depth_frame()
        
        if rgb is not None and depth is not None:
            # Resize depth to match RGB height
            h_rgb = rgb.shape[0]
            scale = h_rgb / depth.shape[0]
            w_depth = int(depth.shape[1] * scale)
            depth = cv2.resize(depth, (w_depth, h_rgb))
            
            # Combine side by side
            combined = np.hstack([rgb, depth])
            return combined
        elif rgb is not None:
            return rgb
        elif depth is not None:
            return depth
        return None

# Global streamer instance
streamer = None

def generate_frames(stream_type='combined'):
    """Generate frames for streaming"""
    global streamer
    
    while True:
        if streamer is None:
            # Waiting screen
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, 'Waiting for camera...', (150, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            if stream_type == 'rgb':
                frame = streamer.get_rgb_frame()
            elif stream_type == 'depth':
                frame = streamer.get_depth_frame()
            else:  # combined
                frame = streamer.get_combined_frame()
            
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, 'No camera data', (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Main page with both streams"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>RealSense Camera Stream</title>
        <style>
            body {
                margin: 0;
                padding: 20px;
                background: #1a1a1a;
                color: white;
                font-family: Arial, sans-serif;
            }
            h1 { text-align: center; }
            .container {
                max-width: 1400px;
                margin: 0 auto;
            }
            .stream {
                text-align: center;
                margin: 20px 0;
            }
            img {
                max-width: 100%;
                border: 2px solid #333;
                border-radius: 8px;
            }
            .controls {
                text-align: center;
                margin: 20px 0;
            }
            a {
                color: #4CAF50;
                text-decoration: none;
                margin: 0 10px;
                padding: 10px 20px;
                border: 1px solid #4CAF50;
                border-radius: 5px;
                display: inline-block;
            }
            a:hover { background: #4CAF50; color: white; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 RealSense Camera Stream</h1>
            <div class="controls">
                <a href="/">Combined View</a>
                <a href="/rgb">RGB Only</a>
                <a href="/depth">Depth Only</a>
            </div>
            <div class="stream">
                <h2>RGB + Depth</h2>
                <img src="/video_combined" />
            </div>
        </div>
    </body>
    </html>
    '''

@app.route('/rgb')
def rgb_page():
    """RGB only page"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>RealSense RGB</title>
        <style>
            body { margin: 0; padding: 20px; background: #1a1a1a; color: white; font-family: Arial; }
            h1 { text-align: center; }
            .stream { text-align: center; margin: 20px 0; }
            img { max-width: 100%; border: 2px solid #333; border-radius: 8px; }
            a { color: #4CAF50; text-decoration: none; }
        </style>
    </head>
    <body>
        <h1>RealSense RGB Camera</h1>
        <p style="text-align: center;"><a href="/">← Back</a></p>
        <div class="stream">
            <img src="/video_rgb" />
        </div>
    </body>
    </html>
    '''

@app.route('/depth')
def depth_page():
    """Depth only page"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>RealSense Depth</title>
        <style>
            body { margin: 0; padding: 20px; background: #1a1a1a; color: white; font-family: Arial; }
            h1 { text-align: center; }
            .stream { text-align: center; margin: 20px 0; }
            img { max-width: 100%; border: 2px solid #333; border-radius: 8px; }
            a { color: #4CAF50; text-decoration: none; }
        </style>
    </head>
    <body>
        <h1>RealSense Depth Camera</h1>
        <p style="text-align: center;"><a href="/">← Back</a></p>
        <div class="stream">
            <img src="/video_depth" />
        </div>
    </body>
    </html>
    '''

@app.route('/video_combined')
def video_combined():
    return Response(generate_frames('combined'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_rgb')
def video_rgb():
    return Response(generate_frames('rgb'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_depth')
def video_depth():
    return Response(generate_frames('depth'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def spin_ros():
    """Spin ROS in separate thread"""
    rclpy.spin(streamer)

def main():
    global streamer
    
    # Check if Flask is available
    try:
        import flask
    except ImportError:
        print('')
        print('ERROR: Flask not found!')
        print('Install it with:')
        print('  pip3 install flask')
        print('')
        return 1
    
    rclpy.init()
    streamer = RealSenseWebStreamer()
    
    # Start ROS spinning in separate thread
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # Get Jetson IP for display
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    
    print('')
    print('=' * 60)
    print('RealSense Web Streamer Running!')
    print('=' * 60)
    print(f'Access from any device on your network:')
    print(f'  http://{local_ip}:5000/')
    print(f'  http://localhost:5000/ (on Jetson)')
    print('')
    print('Press Ctrl+C to stop')
    print('=' * 60)
    print('')
    
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


