#!/usr/bin/env python3
from __future__ import annotations

import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path('/home/user/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf')
OUT = Path('/home/user/PX4-Autopilot/Tools/simulation/gz/worlds/vio_marker_arena.sdf')
BACKUP_DIR = Path('/home/user/PX4-Autopilot/Tools/simulation/gz/world_backups')

ARENA_HALF_SIZE = 10.0
WALL_HEIGHT = 4.0
WALL_THICKNESS = 0.15

COLORS = [
    (0.90, 0.12, 0.12, 1.0),
    (0.12, 0.55, 0.95, 1.0),
    (0.12, 0.80, 0.28, 1.0),
    (0.95, 0.75, 0.10, 1.0),
    (0.70, 0.20, 0.90, 1.0),
    (0.95, 0.35, 0.08, 1.0),
    (0.05, 0.75, 0.75, 1.0),
    (0.95, 0.25, 0.60, 1.0),
    (0.45, 0.85, 0.12, 1.0),
    (0.20, 0.30, 0.95, 1.0),
]

def text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = value
    return child

def material(parent: ET.Element, rgba: tuple[float, float, float, float]) -> None:
    mat = ET.SubElement(parent, 'material')
    value = ' '.join(f'{v:.3f}' for v in rgba)
    text(mat, 'ambient', value)
    text(mat, 'diffuse', value)
    text(mat, 'specular', '0.10 0.10 0.10 1')
    text(mat, 'emissive', '0 0 0 1')

def add_box_model(world, name, pose, size, rgba, collision):
    model = ET.SubElement(world, 'model', {'name': name})
    text(model, 'static', 'true')
    text(model, 'pose', ' '.join(f'{v:.4f}' for v in pose))
    link = ET.SubElement(model, 'link', {'name': 'link'})
    if collision:
        collision_el = ET.SubElement(link, 'collision', {'name': 'collision'})
        geometry = ET.SubElement(collision_el, 'geometry')
        box = ET.SubElement(geometry, 'box')
        text(box, 'size', ' '.join(f'{v:.4f}' for v in size))
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    text(box, 'size', ' '.join(f'{v:.4f}' for v in size))
    material(visual, rgba)

def add_cylinder_visual(world, name, pose, radius, length, rgba):
    model = ET.SubElement(world, 'model', {'name': name})
    text(model, 'static', 'true')
    text(model, 'pose', ' '.join(f'{v:.4f}' for v in pose))
    link = ET.SubElement(model, 'link', {'name': 'link'})
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    geometry = ET.SubElement(visual, 'geometry')
    cylinder = ET.SubElement(geometry, 'cylinder')
    text(cylinder, 'radius', f'{radius:.4f}')
    text(cylinder, 'length', f'{length:.4f}')
    material(visual, rgba)

if not BASE.exists():
    raise FileNotFoundError(f'Base world not found: {BASE}')

BACKUP_DIR.mkdir(parents=True, exist_ok=True)
if OUT.exists():
    backup = BACKUP_DIR / 'vio_marker_arena_previous.sdf'
    shutil.copy2(OUT, backup)
    print(f'Existing arena backed up to: {backup}')

tree = ET.parse(BASE)
root = tree.getroot()
world = root.find('world')
if world is None:
    raise RuntimeError('No <world> element found in default.sdf')

world.set('name', 'vio_marker_arena')
for model in list(world.findall('model')):
    if model.get('name', '').startswith('vio_'):
        world.remove(model)

scene = world.find('scene')
if scene is None:
    scene = ET.SubElement(world, 'scene')
ambient = scene.find('ambient')
if ambient is None:
    ambient = ET.SubElement(scene, 'ambient')
ambient.text = '0.65 0.65 0.65 1'
background = scene.find('background')
if background is None:
    background = ET.SubElement(scene, 'background')
background.text = '0.72 0.78 0.86 1'

wall_color = (0.72, 0.72, 0.72, 1.0)
wall_z = WALL_HEIGHT / 2.0
add_box_model(world, 'vio_wall_north', (0, ARENA_HALF_SIZE, wall_z, 0, 0, 0), (2*ARENA_HALF_SIZE, WALL_THICKNESS, WALL_HEIGHT), wall_color, True)
add_box_model(world, 'vio_wall_south', (0, -ARENA_HALF_SIZE, wall_z, 0, 0, 0), (2*ARENA_HALF_SIZE, WALL_THICKNESS, WALL_HEIGHT), wall_color, True)
add_box_model(world, 'vio_wall_east', (ARENA_HALF_SIZE, 0, wall_z, 0, 0, 0), (WALL_THICKNESS, 2*ARENA_HALF_SIZE, WALL_HEIGHT), wall_color, True)
add_box_model(world, 'vio_wall_west', (-ARENA_HALF_SIZE, 0, wall_z, 0, 0, 0), (WALL_THICKNESS, 2*ARENA_HALF_SIZE, WALL_HEIGHT), wall_color, True)

horizontal_positions = [-8.0, -5.4, -2.7, 0.0, 2.7, 5.4, 8.0]
heights = [0.65, 1.35, 2.15, 3.05]
panel_index = 0
for wall_name, fixed, is_x_wall in [
    ('north', ARENA_HALF_SIZE - 0.081, False),
    ('south', -ARENA_HALF_SIZE + 0.081, False),
    ('east', ARENA_HALF_SIZE - 0.081, True),
    ('west', -ARENA_HALF_SIZE + 0.081, True),
]:
    for row, z in enumerate(heights):
        for col, pos in enumerate(horizontal_positions):
            color = COLORS[(panel_index * 3 + row + col) % len(COLORS)]
            width = 0.65 + 0.12 * ((row + col) % 4)
            height = 0.35 + 0.10 * ((2 * row + col) % 4)
            if not is_x_wall:
                pose = (pos, fixed, z, 0, 0, 0)
                size = (width, 0.035, height)
            else:
                pose = (fixed, pos, z, 0, 0, 0)
                size = (0.035, width, height)
            add_box_model(world, f'vio_panel_{wall_name}_{row}_{col}', pose, size, color, False)
            panel_index += 1

floor_radius = 7.6
for i in range(32):
    angle = 2 * math.pi * i / 32
    x = floor_radius * math.cos(angle)
    y = floor_radius * math.sin(angle)
    color = COLORS[(i * 7) % len(COLORS)]
    sx = 0.55 + 0.12 * (i % 4)
    sy = 0.32 + 0.10 * ((i + 2) % 4)
    add_box_model(world, f'vio_floor_marker_{i:02d}', (x, y, 0.012, 0, 0, angle + math.pi/4), (sx, sy, 0.018), color, False)

add_cylinder_visual(world, 'vio_center_target', (0, 0, 0.012, 0, 0, 0), 0.85, 0.02, (0.95, 0.95, 0.95, 1.0))
for i in range(8):
    angle = 2 * math.pi * i / 8
    x = 1.2 * math.cos(angle)
    y = 1.2 * math.sin(angle)
    add_box_model(world, f'vio_center_marker_{i}', (x, y, 0.013, 0, 0, angle), (0.45, 0.20, 0.02), COLORS[i % len(COLORS)], False)

ET.indent(tree, space='  ')
tree.write(OUT, encoding='utf-8', xml_declaration=True)
print(f'Created: {OUT}')
print('Arena size: 20 m x 20 m')
print('Wall height: 4 m')
print(f'Colored wall panels: {panel_index}')
print('Colored floor markers: 32')
print('Central reference target: yes')
