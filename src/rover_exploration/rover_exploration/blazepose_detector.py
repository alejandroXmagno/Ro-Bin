#!/usr/bin/env python3
"""
BlazePose Person Detection Node
Detects people using MediaPipe BlazePose on RealSense camera feed at 3Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
import json
from threading import Lock


class BlazePoseDetector(Node):
    """ROS2 node for detecting people using MediaPipe BlazePose"""
    
    def __init__(self):
        super().__init__('blazepose_detector')
        
        # Parameters
        self.declare_parameter('detection_rate', 3.0)  # Hz
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('enable_visualization', True)
        
        self.detection_rate = self.get_parameter('detection_rate').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = Lock()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/person_detection/results',
            10
        )
        
        if self.enable_viz:
            self.viz_pub = self.create_publisher(
                Image,
                '/person_detection/visualization',
                10
            )
        
        # Detection timer (3 Hz)
        detection_period = 1.0 / self.detection_rate
        self.detection_timer = self.create_timer(
            detection_period,
            self.detect_person
        )
        
        # Statistics
        self.detection_count = 0
        self.person_detected_count = 0
        
        self.get_logger().info(f'BlazePose Detector initialized')
        self.get_logger().info(f'Detection rate: {self.detection_rate} Hz')
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'Min detection confidence: {self.min_detection_confidence}')
    
    def image_callback(self, msg):
        """Store latest image for processing"""
        with self.image_lock:
            self.latest_image = msg
    
    def detect_person(self):
        """Run pose detection on the latest image"""
        with self.image_lock:
            if self.latest_image is None:
                return
            current_image = self.latest_image
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(current_image, desired_encoding='bgr8')
            
            # Convert BGR to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Run pose detection
            results = self.pose.process(rgb_image)
            
            self.detection_count += 1
            
            # Prepare detection results
            detection_data = {
                'timestamp': self.get_clock().now().to_msg(),
                'person_detected': False,
                'pose_landmarks': None,
                'detection_count': self.detection_count
            }
            
            if results.pose_landmarks:
                self.person_detected_count += 1
                detection_data['person_detected'] = True
                
                # Extract key landmarks
                landmarks = results.pose_landmarks.landmark
                detection_data['pose_landmarks'] = {
                    'nose': {
                        'x': landmarks[self.mp_pose.PoseLandmark.NOSE].x,
                        'y': landmarks[self.mp_pose.PoseLandmark.NOSE].y,
                        'visibility': landmarks[self.mp_pose.PoseLandmark.NOSE].visibility
                    },
                    'left_shoulder': {
                        'x': landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                        'y': landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y,
                        'visibility': landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER].visibility
                    },
                    'right_shoulder': {
                        'x': landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                        'y': landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y,
                        'visibility': landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].visibility
                    },
                    'left_hip': {
                        'x': landmarks[self.mp_pose.PoseLandmark.LEFT_HIP].x,
                        'y': landmarks[self.mp_pose.PoseLandmark.LEFT_HIP].y,
                        'visibility': landmarks[self.mp_pose.PoseLandmark.LEFT_HIP].visibility
                    },
                    'right_hip': {
                        'x': landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP].x,
                        'y': landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP].y,
                        'visibility': landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP].visibility
                    }
                }
                
                # Calculate approximate center position
                nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
                detection_data['person_center'] = {
                    'x': nose.x,
                    'y': nose.y,
                    'pixel_x': int(nose.x * cv_image.shape[1]),
                    'pixel_y': int(nose.y * cv_image.shape[0])
                }
                
                self.get_logger().info(
                    f'Person detected! Position: ({detection_data["person_center"]["pixel_x"]}, '
                    f'{detection_data["person_center"]["pixel_y"]})'
                )
            
            # Publish detection results
            detection_msg = String()
            detection_msg.data = json.dumps(detection_data, default=str)
            self.detection_pub.publish(detection_msg)
            
            # Visualize if enabled
            if self.enable_viz and results.pose_landmarks:
                # Draw pose landmarks on image
                annotated_image = cv_image.copy()
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                )
                
                # Add detection info text
                cv2.putText(
                    annotated_image,
                    f'Person Detected ({self.detection_rate} Hz)',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
                
                cv2.putText(
                    annotated_image,
                    f'Detections: {self.person_detected_count}/{self.detection_count}',
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )
                
                # Publish visualization
                viz_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                viz_msg.header = current_image.header
                self.viz_pub.publish(viz_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in pose detection: {str(e)}')
    
    def destroy_node(self):
        """Cleanup"""
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BlazePoseDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

