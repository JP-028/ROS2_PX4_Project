#!/usr/bin/env python3
from pathlib import Path
import sys

TARGET = Path("scripts/flight/execute_path_from_local_pose.py")

START_MARKER = 'center_offset_x = arc.get("center_offset_x_m", None)'
END_MARKER = 'duration = arc_length / max(self.speed, 1e-6)'

REPLACEMENT = """        center_offset_x = arc.get("center_offset_x_m", None)
        center_offset_y = arc.get("center_offset_y_m", None)

        if center_offset_x is not None and center_offset_y is not None:
            center = self.current_target + np.array([
                float(center_offset_x),
                float(center_offset_y),
                0.0,
            ])
        else:
            center = self.current_target + sign * radius * left

        radial_start = self.current_target - center
        start_angle = math.atan2(radial_start[1], radial_start[0])

        arc_length = radius * angle_rad
        duration = arc_length / max(self.speed, 1e-6)
"""

if not TARGET.exists():
    print(f"[ERROR] File not found: {TARGET}")
    sys.exit(1)

text = TARGET.read_text()

start = text.find(START_MARKER)
if start == -1:
    print(f"[ERROR] Start marker not found: {START_MARKER}")
    sys.exit(1)

# Expand start to the beginning of that line
start = text.rfind("\n", 0, start) + 1

end = text.find(END_MARKER, start)
if end == -1:
    print(f"[ERROR] End marker not found: {END_MARKER}")
    sys.exit(1)

# Expand end to the end of that line
end = text.find("\n", end)
if end == -1:
    end = len(text)
else:
    end += 1

backup = TARGET.with_suffix(TARGET.suffix + ".before_center_offset_fix")
backup.write_text(text)

new_text = text[:start] + REPLACEMENT + text[end:]
TARGET.write_text(new_text)

print(f"[OK] Patched: {TARGET}")
print(f"[OK] Backup:  {backup}")
print()
print("Now run:")
print("  python3 -m py_compile scripts/flight/execute_path_from_local_pose.py")
