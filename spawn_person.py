#!/usr/bin/env python3
"""
Simple script to spawn person-like objects in Gazebo simulation
"""
import subprocess
import sys
import time

def spawn_person(name, x, y, z=0, rotation=0):
    """Spawn a human actor"""
    sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <actor name="{name}">
    <pose>{x} {y} {z} 0 0 {rotation}</pose>
    <skin>
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="standing">
      <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
      <scale>0.055</scale>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <auto_start>true</auto_start>
      <trajectory id="0" type="standing">
        <waypoint>
          <time>0</time>
          <pose>0 0 1.0 0 0 0</pose>
        </waypoint>
        <waypoint>
          <time>30</time>
          <pose>0 0 1.0 0 0 0</pose>
        </waypoint>
      </trajectory>
    </script>
  </actor>
</sdf>"""
    
    # Write to temp file
    temp_file = f"/tmp/{name}.sdf"
    with open(temp_file, 'w') as f:
        f.write(sdf_content)
    
    # Spawn using ign service command
    try:
        cmd = [
            'ign', 'service', '-s', '/world/depot/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'sdf_filename: "{temp_file}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
        ]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and 'data: true' in result.stdout:
            print(f"✅ Spawned {name} at ({x}, {y}, {z})")
            return True
        else:
            print(f"❌ Failed to spawn {name}: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ Error spawning {name}: {e}")
        return False

if __name__ == "__main__":
    print("🤖 Spawning human actors in Gazebo...")
    print("⚠️  Make sure Gazebo is running first!")
    time.sleep(2)
    
    # Spawn multiple people at different locations with different rotations
    spawn_person("person1", 5.0, 0.0, 0.0, rotation=0)
    time.sleep(0.5)
    spawn_person("person2", -3.0, 4.0, 0.0, rotation=1.57)  # 90 degrees
    time.sleep(0.5)
    spawn_person("person3", 2.0, -3.0, 0.0, rotation=3.14)  # 180 degrees
    time.sleep(0.5)
    spawn_person("person4", -2.0, -2.0, 0.0, rotation=4.71)  # 270 degrees
    
    print("\n✅ Done! Check Gazebo window for human actors.")

