#!/usr/bin/env python3
"""
Generator for realistic agricultural simulation world.
Creates 5 garden bed pairs (10 rows total) with natural variation.
"""

import random
import math

random.seed(42)  # reproducible

MESH_URI = 'model://src/my_robot/meshes/bushes.STL'
BASE_SCALE = 0.0002

# World parameters
START_X = -14.0       # robot enters here
END_X = 14.0
PLANT_SPACING = 0.45  # distance between plants along row

# Garden bed layout:
# 5 bed pairs, each pair = 2 parallel rows with ~0.6m between them
# ~2.2m paths between bed pairs
# Layout centered at y=0 (robot navigates bed 0 by default)
#
# bed pair positions (center y):
#   -9.0, -4.5, 0.0, 4.5, 9.0
# row offsets within bed pair: ±0.30 from center
BED_CENTERS = [-9.0, -4.5, 0.0, 4.5, 9.0]
ROW_HALF_GAP = 0.30   # distance from bed center to each row

# Number of plants per row
num_plants = int((END_X - START_X) / PLANT_SPACING) + 1


def plant_x_positions():
    """Generate x positions for plants with small random jitter."""
    positions = []
    for i in range(num_plants):
        x = START_X + i * PLANT_SPACING
        x += random.uniform(-0.05, 0.05)
        positions.append(x)
    return positions


def plant_y_offset(base_y, row_idx, plant_idx):
    """Natural slight curvature + noise in y position."""
    # gentle sine wave for 'not perfectly straight' look
    curve = 0.06 * math.sin(plant_idx * 0.3 + row_idx * 1.1)
    noise = random.gauss(0, 0.04)
    return base_y + curve + noise


def plant_scale():
    """Vary plant size slightly for realism."""
    s = BASE_SCALE * random.uniform(0.85, 1.20)
    return f'{s:.6f} {s:.6f} {s:.6f}'


def plant_green():
    """Slightly varied green color."""
    r = random.uniform(0.05, 0.15)
    g = random.uniform(0.55, 0.75)
    b = random.uniform(0.05, 0.12)
    return (
        f'{r:.2f} {g:.2f} {b:.2f} 1',
        f'{r*2.5:.2f} {min(g*1.4, 1.0):.2f} {b*2.5:.2f} 1'
    )


models = []
model_id = 0

for bed_idx, bed_y in enumerate(BED_CENTERS):
    for side, sign in enumerate([-1, +1]):
        row_y_base = bed_y + sign * ROW_HALF_GAP
        xs = plant_x_positions()
        for plant_idx, x in enumerate(xs):
            y = plant_y_offset(row_y_base, bed_idx * 2 + side, plant_idx)
            z_rot = random.uniform(-0.15, 0.15)  # slight yaw variation
            sc = plant_scale()
            ambient, diffuse = plant_green()
            name = f'bush_{bed_idx}_{side}_{plant_idx}'
            models.append((name, x, y, z_rot, sc, ambient, diffuse))
            model_id += 1


def model_xml(name, x, y, z_rot, sc, ambient, diffuse):
    return f"""    <model name="{name}">
      <pose>{x:.4f} {y:.4f} 0 0 0 {z_rot:.4f}</pose>
      <link name="link">
        <visual name="bush_visual">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>{MESH_URI}</uri>
              <scale>{sc}</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>true</static>
      <self_collide>false</self_collide>
    </model>"""


ground_xml = """    <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>200 200</size>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>50</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>200 200</size>
          </plane>
        </geometry>
        <material>
          <!-- brown soil -->
          <ambient>0.38 0.27 0.12 1</ambient>
          <diffuse>0.50 0.35 0.18 1</diffuse>
          <specular>0.05 0.04 0.02 1</specular>
        </material>
      </visual>
    </link>
  </model>"""

# Add raised bed mounds (box geometry) as visual soil strips under each row pair
bed_strip_xml_list = []
for bed_idx, bed_y in enumerate(BED_CENTERS):
    strip_width = 2 * ROW_HALF_GAP + 0.25  # width of bed
    strip_length = END_X - START_X + 1.0
    strip_name = f'bed_strip_{bed_idx}'
    bed_strip_xml_list.append(f"""    <model name="{strip_name}">
      <pose>{(START_X + END_X) / 2:.2f} {bed_y:.2f} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry>
            <box>
              <size>{strip_length:.2f} {strip_width:.2f} 0.03</size>
            </box>
          </geometry>
          <material>
            <ambient>0.28 0.18 0.08 1</ambient>
            <diffuse>0.35 0.22 0.10 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>true</static>
      <self_collide>false</self_collide>
    </model>""")

sun_xml = """    <light name="sun" type="directional">
      <pose>0 0 20 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.3 0.1 -0.95</direction>
      <diffuse>0.85 0.85 0.80 1</diffuse>
      <specular>0.25 0.25 0.20 1</specular>
      <attenuation>
        <range>1000</range>
        <linear>0.01</linear>
        <constant>0.9</constant>
        <quadratic>0.001</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <light name="ambient_fill" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>0.4</intensity>
      <direction>0.3 -0.1 -0.9</direction>
      <diffuse>0.6 0.7 0.8 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation>
        <range>1000</range>
        <linear>0.01</linear>
        <constant>0.9</constant>
        <quadratic>0.001</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>"""

header = """<?xml version='1.0' encoding='UTF-8'?>
<sdf xmlns:ros2_control="http://playerstage.sourceforge.net/gazebo/xmlschema/#ros2_control" version="1.10">
  <world name="garden_rows_world">
    <physics name="1ms" type="ode">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors" render_engine="ogre2"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.55 0.70 0.85 1</background>
      <shadows>true</shadows>
    </scene>"""

footer = """  </world>
</sdf>"""

lines = [header]
lines.append(ground_xml)
for b in bed_strip_xml_list:
    lines.append(b)
for (name, x, y, z_rot, sc, ambient, diffuse) in models:
    lines.append(model_xml(name, x, y, z_rot, sc, ambient, diffuse))
lines.append(sun_xml)
lines.append(footer)

out = '\n'.join(lines)
outfile = '/home/alexey/ros2_ws/src/my_robot/urdf/garden_rows_world.sdf'
with open(outfile, 'w') as f:
    f.write(out)

print(f"Generated {outfile}")
print(f"  Bed pairs: {len(BED_CENTERS)}")
print(f"  Rows total: {len(BED_CENTERS) * 2}")
print(f"  Plants per row: {num_plants}")
print(f"  Total plants: {len(models)}")
print(f"  Row length: {END_X - START_X:.1f}m")
print(f"  Bed Y centers: {BED_CENTERS}")
