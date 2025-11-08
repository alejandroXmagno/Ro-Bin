#!/usr/bin/env python3
"""
Visualize Filtered Angles as Polar Chart
Shows which angles are being filtered out from LiDAR scans in a polar plot.

Usage:
    python3 visualize_filtered_angles.py <angles_file> [tolerance_degrees]
    
Examples:
    python3 visualize_filtered_angles.py close_obstacles_1762028491_unique_angles.txt
    python3 visualize_filtered_angles.py close_obstacles_1762028491.json 2.0
"""

import json
import math
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import os

# Set backend before importing pyplot
if 'DISPLAY' not in os.environ:
    matplotlib.use('Agg')
    print("Warning: No DISPLAY found, using Agg backend (image saving only)")
else:
    try:
        matplotlib.use('TkAgg')
    except:
        try:
            matplotlib.use('Qt5Agg')
        except:
            pass


def load_angles_from_file(filepath):
    """Load angles from a unique angles text file or JSON file"""
    angles_rad = []
    
    if filepath.endswith('.json'):
        # Load from obstacle recording JSON
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        # Extract unique angles in radians
        unique_angles = set()
        for reading in data['readings']:
            unique_angles.add(reading['angle_rad'])
        angles_rad = list(unique_angles)
        
    elif filepath.endswith('.txt'):
        # Load from unique angles text file
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # Find the radians section
        in_radians_section = False
        for line in lines:
            if 'UNIQUE ANGLES IN RADIANS' in line:
                in_radians_section = True
                continue
            
            if in_radians_section:
                line = line.strip()
                if line == '[':
                    continue
                elif line == ']':
                    break
                elif line and not line.startswith('ANGLE'):
                    # Extract the number (remove comma if present)
                    angle_str = line.rstrip(',').strip()
                    try:
                        angles_rad.append(float(angle_str))
                    except ValueError:
                        pass
    
    return angles_rad


def normalize_angle(angle):
    """Normalize angle to [0, 2π) range"""
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle


def is_angle_filtered(angle, filtered_angles, tolerance):
    """Check if an angle would be filtered given the tolerance"""
    for filtered_angle in filtered_angles:
        # Normalize both angles to [0, 2π)
        angle_norm = normalize_angle(angle)
        filtered_norm = normalize_angle(filtered_angle)
        
        # Calculate angular difference (handling wrap-around)
        diff = abs(angle_norm - filtered_norm)
        diff = min(diff, 2 * math.pi - diff)  # Handle wrap-around
        
        if diff <= tolerance:
            return True
    return False


def create_polar_chart(filtered_angles, tolerance_rad, output_file=None):
    """Create a polar chart showing filtered vs non-filtered angles"""
    
    # Create a fine-grained angle array for smooth visualization
    num_points = 720  # 0.5 degree resolution
    angles = np.linspace(0, 2 * math.pi, num_points, endpoint=False)
    angle_increment = 2 * math.pi / num_points
    
    # Determine which angles are filtered
    is_filtered = np.array([is_angle_filtered(angle, filtered_angles, tolerance_rad) 
                            for angle in angles])
    
    # Create figure with polar projection
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='polar')
    
    # Set up polar plot
    ax.set_theta_zero_location('N')  # 0 degrees at top (north)
    ax.set_theta_direction(-1)  # Clockwise direction
    ax.set_ylim(0, 1.2)
    ax.set_yticklabels([])  # Hide radius labels
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Create a continuous array of colors for each angle
    colors = np.array(['#e74c3c' if f else '#2ecc71' for f in is_filtered])
    
    # Group consecutive regions of the same color
    regions = []
    i = 0
    while i < len(angles):
        region_start_idx = i
        region_color = colors[i]
        region_start_angle = angles[i]
        
        # Find where this region ends (where color changes)
        while i < len(angles) and colors[i] == region_color:
            i += 1
        
        region_end_idx = i
        # The end angle should include the last point of this region
        # For the last region, extend to 2π to close the circle
        if region_end_idx < len(angles):
            # End at the start of the next region (which is the boundary)
            region_end_angle = angles[region_end_idx]
        else:
            # Last region: extend to 2π to close the circle
            region_end_angle = 2 * math.pi
        
        regions.append({
            'start_angle': region_start_angle,
            'end_angle': region_end_angle,
            'color': region_color
        })
    
    # Handle wrap-around: if first and last regions have the same color, merge them
    if len(regions) > 1 and regions[0]['color'] == regions[-1]['color']:
        # Merge: the last region wraps around to connect with the first
        # Adjust the last region to start where it should and extend through the first
        # The merged region starts at the last region's start and ends at the first region's end
        wrap_start = regions[-1]['start_angle']
        wrap_end = regions[0]['end_angle']
        # If wrap_end < wrap_start, it means we need to add 2π
        if wrap_end < wrap_start:
            wrap_end += 2 * math.pi
        regions[-1] = {
            'start_angle': wrap_start,
            'end_angle': wrap_end,
            'color': regions[-1]['color']
        }
        regions = regions[1:]  # Remove the first region (now merged into last)
    
    # Sort regions by start angle to ensure correct drawing order
    regions.sort(key=lambda r: r['start_angle'])
    
    # Verify total coverage (should be 2π)
    total_coverage = 0
    for region in regions:
        start = normalize_angle(region['start_angle'])
        end = region['end_angle']
        if end > 2 * math.pi:
            # Wrap-around region: part from start to 2π, plus part from 0 to (end - 2π)
            total_coverage += (2 * math.pi - start) + (end - 2 * math.pi)
        else:
            end = normalize_angle(end)
            if end < start:
                # This shouldn't happen after normalization, but handle it
                total_coverage += (2 * math.pi - start) + end
            else:
                total_coverage += (end - start)
    
    if abs(total_coverage - 2 * math.pi) > 0.01:
        print(f"Warning: Total coverage is {total_coverage:.6f}, expected {2*math.pi:.6f}")
        print(f"  Number of regions: {len(regions)}")
    
    # Draw each region in order
    for region in regions:
        start_angle = normalize_angle(region['start_angle'])
        end_angle = region['end_angle']
        color = region['color']
        
        # Handle wrap-around: if end_angle > 2π, split into two parts
        if end_angle > 2 * math.pi:
            # First part: from start_angle to 2π
            first_end = 2 * math.pi
            num_points1 = max(int((first_end - start_angle) / 0.001), 10)
            region_angles1 = np.linspace(start_angle, first_end, num_points1, endpoint=True)
            region_radii1 = np.ones_like(region_angles1) * 1.0
            ax.fill_between(region_angles1, 0, region_radii1, 
                           color=color, alpha=0.7, linewidth=0, zorder=2)
            
            # Second part: from 0 to (end_angle - 2π)
            second_start = 0.0
            second_end = normalize_angle(end_angle - 2 * math.pi)
            num_points2 = max(int((second_end - second_start) / 0.001), 10)
            region_angles2 = np.linspace(second_start, second_end, num_points2, endpoint=True)
            region_radii2 = np.ones_like(region_angles2) * 1.0
            ax.fill_between(region_angles2, 0, region_radii2, 
                           color=color, alpha=0.7, linewidth=0, zorder=2)
        else:
            # Normal case: single region
            end_angle = normalize_angle(end_angle)
            num_points = max(int((end_angle - start_angle) / 0.001), 10)
            region_angles = np.linspace(start_angle, end_angle, num_points, endpoint=True)
            region_radii = np.ones_like(region_angles) * 1.0
            
            # Draw the filled region
            ax.fill_between(region_angles, 0, region_radii, 
                           color=color, alpha=0.7, linewidth=0, zorder=2)
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#e74c3c', alpha=0.7, label='Filtered Angles'),
        Patch(facecolor='#2ecc71', alpha=0.7, label='Non-Filtered Angles')
    ]
    ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.15, 1.0), fontsize=12)
    
    # Calculate statistics
    total_angles = len(angles)
    filtered_count = np.sum(is_filtered)
    filtered_percentage = (filtered_count / total_angles * 100) if total_angles > 0 else 0
    
    # Add title with statistics
    title = (
        f'Filtered Angles Visualization\n'
        f'Filtered: {filtered_count}/{total_angles} angles ({filtered_percentage:.1f}%) | '
        f'Tolerance: ±{math.degrees(tolerance_rad):.2f}°'
    )
    ax.set_title(title, pad=30, fontsize=14, fontweight='bold')
    
    # Add angle labels at cardinal directions
    ax.set_thetagrids([0, 90, 180, 270], ['0° (N)', '90° (E)', '180° (S)', '270° (W)'], fontsize=10)
    
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Chart saved to: {output_file}")
    else:
        plt.show()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_filtered_angles.py <angles_file> [tolerance_degrees] [output_file]")
        print("")
        print("Examples:")
        print("  python3 visualize_filtered_angles.py close_obstacles_1762028491_unique_angles.txt")
        print("  python3 visualize_filtered_angles.py close_obstacles_1762028491.json 2.0")
        print("  python3 visualize_filtered_angles.py angles.txt 1.5 filtered_angles.png")
        print("")
        print("This script visualizes which angles are being filtered out from LiDAR scans.")
        print("The polar chart shows:")
        print("  - Red: Angles that are filtered (within tolerance)")
        print("  - Green: Angles that are not filtered")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    if not os.path.exists(input_file):
        print(f"Error: File not found: {input_file}")
        sys.exit(1)
    
    # Load angles
    try:
        filtered_angles = load_angles_from_file(input_file)
        print(f"Loaded {len(filtered_angles)} filtered angles from {input_file}")
        
        if len(filtered_angles) == 0:
            print("Error: No angles found in file")
            sys.exit(1)
        
        # Show angle range
        min_angle = min(filtered_angles)
        max_angle = max(filtered_angles)
        print(f"Angle range: {math.degrees(min_angle):.2f}° to {math.degrees(max_angle):.2f}°")
        
    except Exception as e:
        print(f"Error loading angles from {input_file}: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Get tolerance (default 2.0 degrees)
    tolerance_degrees = 2.0
    if len(sys.argv) > 2:
        try:
            tolerance_degrees = float(sys.argv[2])
        except ValueError:
            print(f"Warning: Invalid tolerance value '{sys.argv[2]}', using default 2.0°")
    
    tolerance_rad = math.radians(tolerance_degrees)
    print(f"Using tolerance: ±{tolerance_degrees}° ({tolerance_rad:.4f} radians)")
    
    # Get output file (optional)
    output_file = None
    if len(sys.argv) > 3:
        output_file = sys.argv[3]
    
    # Create visualization
    print("\nGenerating polar chart...")
    try:
        create_polar_chart(filtered_angles, tolerance_rad, output_file)
        if not output_file:
            print("\nClose the window to exit.")
    except Exception as e:
        print(f"Error creating visualization: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

