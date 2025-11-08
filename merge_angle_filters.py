#!/usr/bin/env python3
"""
Merge multiple angle filter files into one combined filter.
This is useful for filtering multiple types of obstacles (e.g., robot parts + trash bin stands).
"""

import sys
import json

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


def save_merged_angles(angles_rad, output_file):
    """Save merged angles to a text file in the standard format"""
    import math
    
    # Convert to degrees and sort
    angles_deg = sorted([math.degrees(a) for a in angles_rad])
    angles_rad_sorted = sorted(angles_rad)
    
    with open(output_file, 'w') as f:
        f.write('=' * 80 + '\n')
        f.write('MERGED ANGLE FILTERS\n')
        f.write('=' * 80 + '\n')
        f.write(f'Total Unique Angles: {len(angles_rad)}\n')
        f.write('=' * 80 + '\n')
        f.write('\n')
        
        f.write('UNIQUE ANGLES IN DEGREES (sorted):\n')
        f.write('-' * 80 + '\n')
        f.write('[\n')
        for angle in angles_deg:
            f.write(f'  {angle:.6f},\n')
        f.write(']\n')
        f.write('\n')
        
        f.write('UNIQUE ANGLES IN RADIANS (sorted):\n')
        f.write('-' * 80 + '\n')
        f.write('[\n')
        for angle in angles_rad_sorted:
            f.write(f'  {angle:.10f},\n')
        f.write(']\n')
        f.write('\n')
        
        f.write('ANGLE STATISTICS:\n')
        f.write('-' * 80 + '\n')
        f.write(f'Minimum angle: {min(angles_deg):.6f}° ({min(angles_rad_sorted):.10f} rad)\n')
        f.write(f'Maximum angle: {max(angles_deg):.6f}° ({max(angles_rad_sorted):.10f} rad)\n')
        f.write(f'Angular range: {max(angles_deg) - min(angles_deg):.6f}°\n')
        f.write(f'Average angle: {sum(angles_deg) / len(angles_deg):.6f}°\n')
        f.write('=' * 80 + '\n')


def main():
    if len(sys.argv) < 3:
        print('Usage: python3 merge_angle_filters.py <output_file> <input_file1> [input_file2] ...')
        print('')
        print('Examples:')
        print('  # Merge robot parts and trash bin stands:')
        print('  python3 merge_angle_filters.py combined_filters.txt \\')
        print('      close_obstacles_1762028491_unique_angles.txt \\')
        print('      trash_bin_stands_unique_angles.txt')
        print('')
        print('  # Merge from JSON and TXT files:')
        print('  python3 merge_angle_filters.py combined.txt file1.json file2.txt file3.txt')
        print('')
        return 1
    
    output_file = sys.argv[1]
    input_files = sys.argv[2:]
    
    print(f'Merging {len(input_files)} angle filter files...')
    print('')
    
    all_angles = set()
    
    for input_file in input_files:
        try:
            angles = load_angles_from_file(input_file)
            print(f'  ✓ {input_file}: {len(angles)} angles')
            all_angles.update(angles)
        except Exception as e:
            print(f'  ✗ {input_file}: ERROR - {e}')
            return 1
    
    print('')
    print(f'Total unique angles after merging: {len(all_angles)}')
    
    # Save merged angles
    save_merged_angles(list(all_angles), output_file)
    
    print(f'✓ Merged angles saved to: {output_file}')
    print('')
    print('You can now use this file with the filtered LiDAR scanner:')
    print(f'  ./run_filtered_lidar.sh {output_file}')
    print('')
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

