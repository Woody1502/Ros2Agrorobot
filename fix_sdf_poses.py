#!/usr/bin/env python3
"""
Modify garden_rows_world.sdf:
- Move bush pair centers to new y positions (2m spacing)
- Move bed strip y positions accordingly
- Preserve within-pair offsets for bushes
- Only modify the first <pose> after each <model name="..."> tag
"""

import re

SDF_PATH = "/home/Alexey/Ros2Agrorobot/src/my_robot/urdf/garden_rows_world.sdf"

# Old and new center y-positions for each row
ROW_CENTERS = {
    0: {"old": -8.985, "new": -4.0},
    1: {"old": -4.445, "new": -2.0},
    2: {"old": -0.01,  "new":  0.0},
    3: {"old":  4.505, "new":  2.0},
    4: {"old":  9.01,  "new":  4.0},
}

# Bed strip old y -> new y
BED_STRIP_Y = {
    0: {"old": -9.0, "new": -4.0},
    1: {"old": -4.5, "new": -2.0},
    2: {"old":  0.0, "new":  0.0},
    3: {"old":  4.5, "new":  2.0},
    4: {"old":  9.0, "new":  4.0},
}

with open(SDF_PATH, "r") as f:
    content = f.read()

# We'll process line by line to only change the FIRST <pose> after each target <model name="...">
lines = content.splitlines(keepends=True)

# Regex patterns
bush_model_re = re.compile(r'^\s*<model name="bush_(\d+)_(\d+)_(\d+)">\s*$')
bed_strip_model_re = re.compile(r'^\s*<model name="bed_strip_(\d+)">\s*$')
pose_re = re.compile(r'^(\s*<pose>)\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)(\s*</pose>\s*)$')

modified_lines = []
i = 0
bush_changes = 0
bed_changes = 0

while i < len(lines):
    line = lines[i]

    # Check for bush model
    bush_m = bush_model_re.match(line)
    bed_m = bed_strip_model_re.match(line)

    if bush_m:
        row = int(bush_m.group(1))
        modified_lines.append(line)
        i += 1
        # Next line should be the pose — find it (skip any blank lines just in case)
        while i < len(lines):
            next_line = lines[i]
            pose_m = pose_re.match(next_line)
            if pose_m:
                prefix = pose_m.group(1)
                x = float(pose_m.group(2))
                y = float(pose_m.group(3))
                z = float(pose_m.group(4))
                roll = float(pose_m.group(5))
                pitch = float(pose_m.group(6))
                yaw = float(pose_m.group(7))
                suffix = pose_m.group(8)

                old_center = ROW_CENTERS[row]["old"]
                new_center = ROW_CENTERS[row]["new"]
                new_y = y - old_center + new_center

                new_line = f"{prefix}{x:.4f} {new_y:.4f} {z} {roll} {pitch} {yaw}{suffix}"
                modified_lines.append(new_line)
                bush_changes += 1
                i += 1
                break
            else:
                # Not a pose line yet — just append and keep looking
                modified_lines.append(next_line)
                i += 1
        continue

    elif bed_m:
        strip_id = int(bed_m.group(1))
        modified_lines.append(line)
        i += 1
        # Next line should be the pose
        while i < len(lines):
            next_line = lines[i]
            pose_m = pose_re.match(next_line)
            if pose_m:
                prefix = pose_m.group(1)
                x = float(pose_m.group(2))
                y = float(pose_m.group(3))
                z = float(pose_m.group(4))
                roll = float(pose_m.group(5))
                pitch = float(pose_m.group(6))
                yaw = float(pose_m.group(7))
                suffix = pose_m.group(8)

                new_y = BED_STRIP_Y[strip_id]["new"]

                new_line = f"{prefix}{x:.2f} {new_y:.2f} {z} {roll} {pitch} {yaw}{suffix}"
                modified_lines.append(new_line)
                bed_changes += 1
                i += 1
                break
            else:
                modified_lines.append(next_line)
                i += 1
        continue

    else:
        modified_lines.append(line)
        i += 1

print(f"Bush pose changes: {bush_changes}")
print(f"Bed strip pose changes: {bed_changes}")

output = "".join(modified_lines)

with open(SDF_PATH, "w") as f:
    f.write(output)

print("File written successfully.")
