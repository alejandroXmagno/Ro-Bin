#!/bin/bash

# View RealSense camera in RViz
# Works great for remote viewing over network

echo "=========================================="
echo "Launching RViz with RealSense Camera"
echo "=========================================="
echo ""
echo "RViz will show:"
echo "  - RGB camera feed"
echo "  - Depth camera feed"
echo "  - Point cloud (3D view)"
echo ""
echo "Add displays as needed in RViz interface"
echo ""

# Create temporary RViz config if it doesn't exist
RVIZ_CONFIG="realsense_camera.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "Creating RViz configuration..."
    cat > "$RVIZ_CONFIG" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_common/Group
      Name: RealSense Camera
      Displays:
        - Class: rviz_default_plugins/Image
          Name: RGB Image
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /camera/camera/color/image_raw
          Value: true
        - Class: rviz_default_plugins/Image
          Name: Depth Image
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /camera/camera/depth/image_rect_raw
          Value: true
        - Class: rviz_default_plugins/PointCloud2
          Name: Point Cloud
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /camera/camera/depth/color/points
          Value: true
  Global Options:
    Fixed Frame: camera_link
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
EOF
    echo "  ✓ Created $RVIZ_CONFIG"
    echo ""
fi

# Launch RViz
rviz2 -d "$RVIZ_CONFIG"


