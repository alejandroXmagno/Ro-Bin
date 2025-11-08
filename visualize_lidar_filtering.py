#!/usr/bin/env python3
"""
LiDAR Filtering Visualization
Visualizes the angular distribution of used vs filtered LiDAR points as a polar pie chart.

Subscribes to:
- /scan (original LiDAR)
- /scan_filtered (filtered LiDAR)

Displays:
- Polar plot (pie chart from above) showing angular distribution:
  - Green: Angles where points are used
  - Red: Angles where points are filtered
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib
# Set backend before importing pyplot
import os
if 'DISPLAY' not in os.environ:
    # Try to use a non-GUI backend if no display is available
    matplotlib.use('Agg')  # Use Agg backend if no display
    print("Warning: No DISPLAY found, using Agg backend (image saving only)")
else:
    # Try TkAgg first (most compatible), fall back to Qt5Agg
    try:
        matplotlib.use('TkAgg')
    except:
        try:
            matplotlib.use('Qt5Agg')
        except:
            pass  # Use default

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import threading
import time


class LidarFilteringVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_filtering_visualizer')
        
        # Data storage (thread-safe)
        self.lock = threading.Lock()
        self.last_original_scan = None
        self.last_filtered_scan = None
        self.scan_count = 0
        
        # Angular distribution data
        self.angle_data = {
            'angles': [],      # List of angles in radians
            'is_filtered': [], # List of booleans: True if filtered, False if used
            'angle_increment': 0.0  # Angle increment from scan
        }
        
        # Statistics
        self.stats = {
            'total_points': 0,
            'used_points': 0,
            'filtered_points': 0
        }
        
        # Subscribers
        self.original_sub = self.create_subscription(
            LaserScan, '/scan', self.original_callback, 10
        )
        
        self.filtered_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.filtered_callback, 10
        )
        
        self.get_logger().info('LiDAR Filtering Visualizer initialized')
        self.get_logger().info('Waiting for scan data...')
        self.get_logger().info('Subscribed to: /scan and /scan_filtered')
        self.last_log_time = time.time()
        
        # Matplotlib setup - single polar plot
        # Use non-blocking mode but with proper event loop
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.fig.suptitle('LiDAR Filtering Visualization - Angular Distribution', 
                          fontsize=16, fontweight='bold')
        
        # Initialize polar plot settings
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_ylim(0, 1.2)
        self.ax.set_yticklabels([])
        self.ax.grid(True, alpha=0.3)
        self.ax.set_thetalim(0, 2 * math.pi)
        
        # Start animation (interval in milliseconds)
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=False
        )
    
    def original_callback(self, msg):
        """Store original scan data"""
        with self.lock:
            self.last_original_scan = msg
            self.process_scans()
    
    def filtered_callback(self, msg):
        """Store filtered scan data"""
        with self.lock:
            self.last_filtered_scan = msg
            self.process_scans()
    
    def process_scans(self):
        """Compare original and filtered scans to calculate angular distribution"""
        if self.last_original_scan is None or self.last_filtered_scan is None:
            return
        
        original = self.last_original_scan
        filtered = self.last_filtered_scan
        
        # Ensure scans are synchronized (same number of points)
        if len(original.ranges) != len(filtered.ranges):
            return
        
        self.scan_count += 1
        
        # Reset data
        angles = []
        is_filtered_list = []
        
        # Count points
        total_points = len(original.ranges)
        filtered_points = 0
        used_points = 0
        
        angle_increment = original.angle_increment
        current_angle = original.angle_min
        
        for i in range(total_points):
            original_range = original.ranges[i]
            filtered_range = filtered.ranges[i]
            
            # Check if point is filtered
            # In filtered_lidar_scanner, filtered points are set to inf
            # A point is filtered if: filtered is inf AND original is NOT inf
            original_is_inf = math.isinf(original_range)
            filtered_is_inf = math.isinf(filtered_range)
            is_filtered = filtered_is_inf and not original_is_inf
            
            angles.append(current_angle)
            is_filtered_list.append(is_filtered)
            
            if is_filtered:
                filtered_points += 1
            else:
                used_points += 1
            
            current_angle += angle_increment
        
        # Update data
        with self.lock:
            self.angle_data['angles'] = angles
            self.angle_data['is_filtered'] = is_filtered_list
            self.angle_data['angle_increment'] = angle_increment
            self.stats['total_points'] = total_points
            self.stats['used_points'] = used_points
            self.stats['filtered_points'] = filtered_points
        
        # Log occasionally for debugging
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:  # Every 2 seconds
            self.get_logger().info(
                f'Processed scan #{self.scan_count}: '
                f'{used_points} used, {filtered_points} filtered '
                f'({len(angles)} total angles)'
            )
            self.last_log_time = current_time
    
    def update_plot(self, frame):
        """Update the polar plot showing angular distribution"""
        with self.lock:
            angles = self.angle_data['angles'].copy()
            is_filtered = self.angle_data['is_filtered'].copy()
            angle_increment = self.angle_data.get('angle_increment', 0.01)
            stats = self.stats.copy()
            scan_count = self.scan_count
        
        # Clear axis
        self.ax.clear()
        
        # Prepare data
        if len(angles) == 0:
            # Use polar coordinates for text placement
            self.ax.text(0, 0.5, 'Waiting for scan data...', 
                         ha='center', va='center', fontsize=14,
                         transform=self.ax.transAxes)
            self.ax.set_ylim(0, 1.2)
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            return
        
        # Ensure valid angle increment
        if angle_increment == 0:
            angle_increment = 0.01
        
        # Group consecutive filtered/used angles into wedges
        # This creates a pie chart where each wedge represents a continuous angular region
        current_state = None
        wedge_starts = []
        wedge_sizes = []
        wedge_colors = []
        
        for i in range(len(angles)):
            state = is_filtered[i]
            angle = angles[i]
            
            if current_state is None:
                # First angle
                current_state = state
                wedge_starts.append(angle)
                wedge_sizes.append(angle_increment)
                wedge_colors.append('#e74c3c' if state else '#2ecc71')  # Red if filtered, Green if used
            elif state == current_state:
                # Same state, extend current wedge
                wedge_sizes[-1] += angle_increment
            else:
                # State changed, start new wedge
                current_state = state
                wedge_starts.append(angle)
                wedge_sizes.append(angle_increment)
                wedge_colors.append('#e74c3c' if state else '#2ecc71')
        
        # Check if we have any wedges
        if len(wedge_starts) == 0:
            self.ax.text(0, 0.5, f'No data to display\n({len(angles)} angles found)', 
                         ha='center', va='center', fontsize=12,
                         transform=self.ax.transAxes)
            self.ax.set_ylim(0, 1.2)
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            return
        
        # Create pie chart wedges using bar chart in polar coordinates
        # Adjust angles so 0 degrees is at the top (north)
        offset_angle = math.pi / 2  # 90 degrees offset to put 0 at top
        
        # Draw each wedge as a filled arc
        for i in range(len(wedge_starts)):
            start = wedge_starts[i] + offset_angle
            size = wedge_sizes[i]
            color = wedge_colors[i]
            
            # Create theta array for smooth arc
            num_points = max(int(size / 0.001), 10)  # At least 10 points per wedge
            theta = np.linspace(start, start + size, num_points)
            radius = np.ones_like(theta)  # Constant radius for pie chart effect
            
            # Fill the wedge from center (0) to edge (radius)
            self.ax.fill_between(theta, 0, radius, color=color, alpha=0.8, linewidth=0.5)
        
        # Force redraw
        self.fig.canvas.draw_idle()
        
        # Set up polar plot (always configure these)
        self.ax.set_theta_zero_location('N')  # 0 degrees at top
        self.ax.set_theta_direction(-1)  # Clockwise (typical for polar plots)
        self.ax.set_ylim(0, 1.2)  # Set radius limit
        self.ax.set_yticklabels([])  # Hide radius labels
        self.ax.grid(True, alpha=0.3)
        
        # Always set theta limits to show full circle
        self.ax.set_thetalim(0, 2 * math.pi)
        
        # Add legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='#2ecc71', alpha=0.7, label='Used Angles'),
            Patch(facecolor='#e74c3c', alpha=0.7, label='Filtered Angles')
        ]
        self.ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.2, 1.0))
        
        # Add title with statistics
        used_count = stats['used_points']
        filtered_count = stats['filtered_points']
        total_count = stats['total_points']
        used_pct = (used_count / total_count * 100) if total_count > 0 else 0
        filtered_pct = (filtered_count / total_count * 100) if total_count > 0 else 0
        
        title = (
            f'Angular Distribution (Scan #{scan_count})\n'
            f'Used: {used_count} ({used_pct:.1f}%) | Filtered: {filtered_count} ({filtered_pct:.1f}%)'
        )
        self.ax.set_title(title, pad=20, fontsize=12, fontweight='bold')
        
        plt.tight_layout()


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 70)
    print("LiDAR Filtering Visualization")
    print("=" * 70)
    print("This tool visualizes the area/coverage of used vs filtered LiDAR points.")
    print("")
    print("Requirements:")
    print("  - Original LiDAR data on /scan")
    print("  - Filtered LiDAR data on /scan_filtered")
    print("")
    print("Make sure filtered_lidar_scanner.py is running!")
    print("=" * 70)
    print("")
    
    visualizer = LidarFilteringVisualizer()
    
    # Run ROS node in a separate thread
    def spin_node():
        try:
            rclpy.spin(visualizer)
        except Exception as e:
            print(f"Error in ROS spin: {e}")
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    print("Starting visualization window...")
    print("Close the window or press Ctrl+C to exit")
    
    # Check if we can display
    backend = matplotlib.get_backend()
    print(f"Using matplotlib backend: {backend}")
    
    if backend.lower() == 'agg':
        print("ERROR: Cannot display window - no display available!")
        print("This typically happens when running over SSH without X11 forwarding.")
        print("Try: ssh -X user@host (for X11 forwarding)")
        visualizer.destroy_node()
        rclpy.shutdown()
        return
    
    # Show matplotlib plot (this blocks until window is closed)
    try:
        # Ensure the figure is shown
        plt.show(block=True)  # Explicitly use blocking mode
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error displaying plot: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        visualizer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

