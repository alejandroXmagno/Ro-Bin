#!/usr/bin/env python3

import json
import sys

def extract_unique_angles(json_file):
    """Extract unique angles from obstacle recording JSON file"""
    
    # Read the JSON file
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # Extract unique angles (use a set for uniqueness)
    unique_angles_rad = set()
    unique_angles_deg = set()
    
    for reading in data['readings']:
        unique_angles_rad.add(reading['angle_rad'])
        unique_angles_deg.add(reading['angle_deg'])
    
    # Convert to sorted lists
    unique_angles_rad_list = sorted(list(unique_angles_rad))
    unique_angles_deg_list = sorted(list(unique_angles_deg))
    
    # Create output filename
    output_file = json_file.replace('.json', '_unique_angles.txt')
    
    # Write to text file
    with open(output_file, 'w') as f:
        f.write('='*80 + '\n')
        f.write('UNIQUE ANGLES EXTRACTED FROM OBSTACLE RECORDING\n')
        f.write('='*80 + '\n')
        f.write(f'Source File: {json_file}\n')
        f.write(f'Total Readings: {len(data["readings"])}\n')
        f.write(f'Unique Angles Found: {len(unique_angles_deg_list)}\n')
        f.write('='*80 + '\n\n')
        
        # Write angles in degrees (array format)
        f.write('UNIQUE ANGLES IN DEGREES (sorted):\n')
        f.write('-'*80 + '\n')
        f.write('[\n')
        for i, angle in enumerate(unique_angles_deg_list):
            f.write(f'  {angle:.6f}')
            if i < len(unique_angles_deg_list) - 1:
                f.write(',\n')
            else:
                f.write('\n')
        f.write(']\n\n')
        
        # Write angles in radians (array format)
        f.write('UNIQUE ANGLES IN RADIANS (sorted):\n')
        f.write('-'*80 + '\n')
        f.write('[\n')
        for i, angle in enumerate(unique_angles_rad_list):
            f.write(f'  {angle:.10f}')
            if i < len(unique_angles_rad_list) - 1:
                f.write(',\n')
            else:
                f.write('\n')
        f.write(']\n\n')
        
        # Write summary statistics
        f.write('ANGLE STATISTICS:\n')
        f.write('-'*80 + '\n')
        f.write(f'Minimum angle: {min(unique_angles_deg_list):.6f}° ({min(unique_angles_rad_list):.10f} rad)\n')
        f.write(f'Maximum angle: {max(unique_angles_deg_list):.6f}° ({max(unique_angles_rad_list):.10f} rad)\n')
        f.write(f'Angular range: {max(unique_angles_deg_list) - min(unique_angles_deg_list):.6f}°\n')
        f.write(f'Average angle: {sum(unique_angles_deg_list)/len(unique_angles_deg_list):.6f}°\n')
        f.write('\n')
        
        # Write angle distribution
        f.write('ANGLE DISTRIBUTION:\n')
        f.write('-'*80 + '\n')
        
        # Count readings per angle
        angle_counts = {}
        for reading in data['readings']:
            angle = reading['angle_deg']
            angle_counts[angle] = angle_counts.get(angle, 0) + 1
        
        # Sort by count (descending)
        sorted_counts = sorted(angle_counts.items(), key=lambda x: x[1], reverse=True)
        
        f.write('Top 10 most frequent angles:\n')
        for i, (angle, count) in enumerate(sorted_counts[:10]):
            f.write(f'  {i+1}. {angle:8.3f}° : {count:4d} readings ({count/len(data["readings"])*100:5.2f}%)\n')
        
    print(f'✓ Unique angles extracted successfully!')
    print(f'  Total readings: {len(data["readings"])}')
    print(f'  Unique angles: {len(unique_angles_deg_list)}')
    print(f'  Output saved to: {output_file}')
    
    return output_file


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 extract_unique_angles.py <json_file>')
        sys.exit(1)
    
    json_file = sys.argv[1]
    extract_unique_angles(json_file)

