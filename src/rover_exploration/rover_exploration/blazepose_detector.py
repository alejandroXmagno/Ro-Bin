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

        # Depth camera topic
        self.depth_topic = '/camera/depth/image_rect_raw'
        self.depth_image = None
        self.depth_lock = Lock()
        
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
        
        # Depth image subscription
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
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
        self.waving_detected_count = 0
        
        # Hand waving detection state
        self.previous_hand_positions = {}  # person_id -> [(x, y, timestamp), ...]
        
        self.get_logger().info(f'BlazePose Detector initialized (Hand Waving Detection)')
        self.get_logger().info(f'Detection rate: {self.detection_rate} Hz')
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'Min detection confidence: {self.min_detection_confidence}')
    
    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            with self.depth_lock:
                # Convert ROS depth image to cv2 format (16-bit or 32-bit float)
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # Log depth image stats occasionally
                if hasattr(self, '_depth_log_count'):
                    self._depth_log_count += 1
                else:
                    self._depth_log_count = 0
                
                if self._depth_log_count % 30 == 0:  # Every 30 frames (~10s at 3Hz)
                    import numpy as np
                    valid_depths = self.depth_image[self.depth_image > 0]
                    if len(valid_depths) > 0:
                        self.get_logger().info(f'📷 Depth image: shape={self.depth_image.shape}, '
                                             f'min={np.min(valid_depths):.2f}, max={np.max(valid_depths):.2f}, '
                                             f'mean={np.mean(valid_depths):.2f}')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')
    
    def image_callback(self, msg):
        """Store latest image for processing"""
        with self.image_lock:
            self.latest_image = msg
    
    def detect_hand_waving(self, landmarks):
        """Detect if a person is waving their hand (simplified for simulation)"""
        # For simulation, detect any person with visible hands as "waving"
        # This is because Gazebo actors have limited animations
        left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        
        # Simplified: Hand is "waving" if wrist is at or above shoulder level
        # (more lenient for simulation)
        left_hand_up = (
            left_wrist.visibility > 0.3 and
            left_shoulder.visibility > 0.3 and
            left_wrist.y < left_shoulder.y + 0.15  # Hand near or above shoulder
        )
        
        right_hand_up = (
            right_wrist.visibility > 0.3 and
            right_shoulder.visibility > 0.3 and
            right_wrist.y < right_shoulder.y + 0.15  # Hand near or above shoulder
        )
        
        # Person is waving if either hand is raised enough
        is_waving = left_hand_up or right_hand_up
        
        if is_waving:
            hand_position = left_wrist if left_hand_up else right_wrist
            return True, (hand_position.x, hand_position.y)
        
        return False, None
    
    def get_depth_at_pixel(self, pixel_x, pixel_y):
        """Get depth value at a specific pixel location"""
        with self.depth_lock:
            if self.depth_image is None:
                self.get_logger().warn('⚠️  Depth image is None!')
                return None
            
            # Ensure pixel coordinates are within bounds
            h, w = self.depth_image.shape[:2]
            pixel_x = max(0, min(pixel_x, w-1))
            pixel_y = max(0, min(pixel_y, h-1))
            
            # Sample a small region around the pixel for robustness
            region_size = 5
            x_start = max(0, pixel_x - region_size)
            x_end = min(w, pixel_x + region_size)
            y_start = max(0, pixel_y - region_size)
            y_end = min(h, pixel_y + region_size)
            
            depth_region = self.depth_image[y_start:y_end, x_start:x_end]
            
            # Filter out zero/invalid depths and get median
            valid_depths = depth_region[depth_region > 0]
            if len(valid_depths) > 0:
                depth_meters = np.median(valid_depths) / 1000.0  # Convert mm to meters
                return float(depth_meters)
            
            return None
    
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
                'is_waving': False,
                'pose_landmarks': None,
                'detection_count': self.detection_count
            }
            
            if results.pose_landmarks:
                self.person_detected_count += 1
                detection_data['person_detected'] = True
                
                # Detect hand waving
                is_waving, hand_pos = self.detect_hand_waving(results.pose_landmarks.landmark)
                detection_data['is_waving'] = is_waving
                
                if is_waving:
                    self.waving_detected_count += 1
                
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
                pixel_x = int(nose.x * cv_image.shape[1])
                pixel_y = int(nose.y * cv_image.shape[0])
                
                # Get depth at person's center
                depth = self.get_depth_at_pixel(pixel_x, pixel_y)
                if depth is None or depth <= 0:
                    self.get_logger().warn(f'⚠️  Invalid depth at pixel ({pixel_x}, {pixel_y}): {depth}')
                
                detection_data['person_center'] = {
                    'x': nose.x,
                    'y': nose.y,
                    'pixel_x': pixel_x,
                    'pixel_y': pixel_y,
                    'depth': depth  # Depth in meters
                }
                
                wave_status = "👋 WAVING!" if detection_data['is_waving'] else ""
                depth_str = f"{depth:.2f}m" if depth else "unknown"
                self.get_logger().info(
                    f'Person detected! {wave_status} Position: ({pixel_x}, {pixel_y}), Depth: {depth_str}'
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
                status_text = 'Person Detected - WAVING!' if detection_data.get('is_waving', False) else 'Person Detected'
                color = (0, 255, 255) if detection_data.get('is_waving', False) else (0, 255, 0)  # Yellow if waving
                
                cv2.putText(
                    annotated_image,
                    status_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2
                )
                
                cv2.putText(
                    annotated_image,
                    f'Detections: {self.person_detected_count}/{self.detection_count} | Waving: {self.waving_detected_count}',
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

