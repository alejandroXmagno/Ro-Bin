#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
from collections import defaultdict
import json

class CloseObstacleRecorder(Node):
    def __init__(self, duration=5.0, distance_threshold=0.3048):  # 1 foot = 0.3048 meters
        super().__init__('close_obstacle_recorder')
        
        self.duration = duration
        self.distance_threshold = distance_threshold
        self.start_time = None
        self.recording = False
        self.close_readings = []  # List of (timestamp, angle, distance) tuples
        self.angle_bins = defaultdict(list)  # Group readings by angle bins
        
        # Subscribe to laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.get_logger().info(f'Close Obstacle Recorder initialized')
        self.get_logger().info(f'Recording obstacles closer than {distance_threshold:.3f}m (1 foot)')
        self.get_logger().info(f'Recording duration: {duration} seconds')
        self.get_logger().info(f'Waiting for first scan...')
        
    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        
        # Start recording on first message
        if self.start_time is None:
            self.start_time = current_time
            self.recording = True
            self.get_logger().info('Recording started!')
        
        # Check if recording period is complete
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        if elapsed >= self.duration:
            if self.recording:
                self.recording = False
                self.process_and_display_results(msg)
                self.get_logger().info('Recording complete! Shutting down...')
                rclpy.shutdown()
            return
        
        # Process scan data
        if self.recording:
            angle = msg.angle_min
            for i, range_value in enumerate(msg.ranges):
                # Check if reading is valid and within threshold
                if msg.range_min <= range_value <= self.distance_threshold:
                    # Convert angle to degrees for easier interpretation
                    angle_deg = math.degrees(angle)
                    
                    # Normalize angle to [-180, 180]
                    if angle_deg > 180:
                        angle_deg -= 360
                    
                    # Record the reading
                    self.close_readings.append({
                        'timestamp': elapsed,
                        'angle_rad': angle,
                        'angle_deg': angle_deg,
                        'distance_m': range_value,
                        'distance_ft': range_value * 3.28084
                    })
                    
                    # Group by 10-degree bins for summary
                    angle_bin = int(angle_deg / 10) * 10
                    self.angle_bins[angle_bin].append(range_value)
                
                angle += msg.angle_increment
            
            # Progress update
            if int(elapsed * 2) % 2 == 0:  # Every 0.5 seconds
                self.get_logger().info(f'Recording: {elapsed:.1f}s / {self.duration:.1f}s - '
                                     f'Close readings: {len(self.close_readings)}')
    
    def process_and_display_results(self, last_msg):
        """Process and display the recorded data"""
        self.get_logger().info('')
        self.get_logger().info('=' * 80)
        self.get_logger().info('CLOSE OBSTACLE RECORDING RESULTS')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Recording Duration: {self.duration} seconds')
        self.get_logger().info(f'Distance Threshold: {self.distance_threshold:.3f}m (1 foot)')
        self.get_logger().info(f'Total Close Readings: {len(self.close_readings)}')
        self.get_logger().info('')
        
        if len(self.close_readings) == 0:
            self.get_logger().info('No obstacles detected within 1 foot during the recording period.')
            return
        
        # Calculate statistics
        distances = [r['distance_m'] for r in self.close_readings]
        angles = [r['angle_deg'] for r in self.close_readings]
        
        min_dist = min(distances)
        max_dist = max(distances)
        avg_dist = sum(distances) / len(distances)
        
        self.get_logger().info('DISTANCE STATISTICS:')
        self.get_logger().info(f'  Closest obstacle: {min_dist:.3f}m ({min_dist * 3.28084:.2f} ft)')
        self.get_logger().info(f'  Farthest (within threshold): {max_dist:.3f}m ({max_dist * 3.28084:.2f} ft)')
        self.get_logger().info(f'  Average distance: {avg_dist:.3f}m ({avg_dist * 3.28084:.2f} ft)')
        self.get_logger().info('')
        
        # Display angular distribution
        self.get_logger().info('ANGULAR DISTRIBUTION (10-degree bins):')
        self.get_logger().info('  Angle Range  | Count | Avg Distance')
        self.get_logger().info('  ' + '-' * 45)
        
        sorted_bins = sorted(self.angle_bins.items())
        for angle_bin, distances in sorted_bins:
            avg_bin_dist = sum(distances) / len(distances)
            self.get_logger().info(f'  {angle_bin:4d}° to {angle_bin+10:4d}° | {len(distances):5d} | '
                                 f'{avg_bin_dist:.3f}m ({avg_bin_dist * 3.28084:.2f} ft)')
        
        self.get_logger().info('')
        
        # Find most common directions
        self.get_logger().info('OBSTACLE DIRECTIONS:')
        if sorted_bins:
            max_bin = max(sorted_bins, key=lambda x: len(x[1]))
            self.get_logger().info(f'  Most obstacles at: {max_bin[0]}° to {max_bin[0]+10}° '
                                 f'({len(max_bin[1])} readings)')
        
        # Direction labels
        directions = {
            'Front (±30°)': [r for r in self.close_readings if abs(r['angle_deg']) <= 30],
            'Left Side (60° to 120°)': [r for r in self.close_readings if 60 <= r['angle_deg'] <= 120],
            'Right Side (-120° to -60°)': [r for r in self.close_readings if -120 <= r['angle_deg'] <= -60],
            'Rear (150° to 180° or -180° to -150°)': [r for r in self.close_readings 
                                                       if abs(r['angle_deg']) >= 150],
        }
        
        self.get_logger().info('')
        self.get_logger().info('READINGS BY DIRECTION:')
        for direction, readings in directions.items():
            if readings:
                avg = sum(r['distance_m'] for r in readings) / len(readings)
                self.get_logger().info(f'  {direction}: {len(readings)} readings, '
                                     f'avg {avg:.3f}m ({avg * 3.28084:.2f} ft)')
        
        # Save to file
        output_file = f'close_obstacles_{int(time.time())}.json'
        with open(output_file, 'w') as f:
            json.dump({
                'metadata': {
                    'duration_seconds': self.duration,
                    'threshold_meters': self.distance_threshold,
                    'threshold_feet': self.distance_threshold * 3.28084,
                    'total_readings': len(self.close_readings),
                    'timestamp': time.time()
                },
                'statistics': {
                    'min_distance_m': min_dist,
                    'max_distance_m': max_dist,
                    'avg_distance_m': avg_dist,
                    'min_distance_ft': min_dist * 3.28084,
                    'max_distance_ft': max_dist * 3.28084,
                    'avg_distance_ft': avg_dist * 3.28084,
                },
                'angular_distribution': {str(k): len(v) for k, v in sorted_bins},
                'direction_summary': {k: len(v) for k, v in directions.items()},
                'readings': self.close_readings
            }, f, indent=2)
        
        self.get_logger().info('')
        self.get_logger().info(f'Detailed data saved to: {output_file}')
        self.get_logger().info('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments for duration and distance
    import sys
    duration = 5.0
    distance = 0.3048  # 1 foot in meters
    
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            print(f"Invalid duration: {sys.argv[1]}, using default 5.0 seconds")
    
    if len(sys.argv) > 2:
        try:
            distance = float(sys.argv[2])
        except ValueError:
            print(f"Invalid distance: {sys.argv[2]}, using default 0.3048m (1 foot)")
    
    recorder = CloseObstacleRecorder(duration=duration, distance_threshold=distance)
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

