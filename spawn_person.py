#!/usr/bin/env python3
"""
Spawn waving people in Gazebo simulation with depth markers
"""
import subprocess
import time

def spawn_person(name, x, y, z=0, rotation=0, waving=True):
    """Spawn a human actor with physical depth marker"""
    # Spawn visual actor
    animation_file = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae"
    animation_name = "standing"
    
    actor_sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <actor name="{name}">
    <pose>{x} {y} {z} 0 0 {rotation}</pose>
    <skin>
      <filename>{animation_file}</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="{animation_name}">
      <filename>{animation_file}</filename>
      <scale>0.055</scale>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <auto_start>true</auto_start>
      <trajectory id="0" type="{animation_name}">
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
    
    # Write actor SDF to temp file
    with open(f'/tmp/{name}_actor.sdf', 'w') as f:
        f.write(actor_sdf)
    
    # Spawn actor
    result = subprocess.run([
        'ign', 'service', '-s', '/world/depot/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req',
        f'sdf_filename: "/tmp/{name}_actor.sdf"'
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"✅ Spawned actor: {name} at ({x}, {y})")
    else:
        print(f"❌ Failed to spawn actor {name}: {result.stderr}")
    
    time.sleep(0.5)
    
    # Now spawn physical depth marker (sphere at chest height)
    marker_name = f"{name}_depth_marker"
    chest_z = z + 1.4  # Chest height for detection
    
    marker_sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{marker_name}">
    <pose>{x} {y} {chest_z} 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.25</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.25</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 0.1</ambient>
          <diffuse>1 0 0 0.1</diffuse>
          <specular>1 0 0 0.1</specular>
        </material>
        <transparency>0.95</transparency>
      </visual>
      <sensor name="depth_sensor" type="depth_camera">
        <update_rate>30</update_rate>
        <always_on>1</always_on>
      </sensor>
    </link>
  </model>
</sdf>"""
    
    # Write marker SDF to temp file
    with open(f'/tmp/{marker_name}.sdf', 'w') as f:
        f.write(marker_sdf)
    
    # Spawn marker
    result = subprocess.run([
        'ign', 'service', '-s', '/world/depot/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req',
        f'sdf_filename: "/tmp/{marker_name}.sdf"'
    ], capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"   ✅ Added depth marker for {name}")
    else:
        print(f"   ⚠️  Depth marker may have issues: {result.stderr}")

def remove_person(name):
    """Remove both actor and depth marker"""
    # Remove actor
    subprocess.run([
        'ign', 'service', '-s', '/world/depot/remove',
        '--reqtype', 'ignition.msgs.Entity',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '1000',
        '--req',
        f'name: "{name}", type: 2'
    ], capture_output=True)
    
    # Remove depth marker
    marker_name = f"{name}_depth_marker"
    subprocess.run([
        'ign', 'service', '-s', '/world/depot/remove',
        '--reqtype', 'ignition.msgs.Entity',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '1000',
        '--req',
        f'name: "{marker_name}", type: 2'
    ], capture_output=True)

def main():
    print("🚶 Spawning waving people with depth markers...")
    
    # Remove any existing people first
    for i in range(1, 5):
        remove_person(f'person{i}')
    
    time.sleep(1)
    
    # Spawn 4 people at different locations with depth markers
    spawn_person('person1', 3.0, 2.0, waving=True)
    spawn_person('person2', -2.0, 4.0, waving=True)
    spawn_person('person3', 5.0, -3.0, waving=True)
    spawn_person('person4', -4.0, -2.0, waving=True)
    
    print("\n✅ Spawned 4 waving people with depth markers!")
    print("   The semi-transparent red spheres provide depth data")
    print("   while the animated actors provide visuals for BlazePose")

if __name__ == '__main__':
    main()
