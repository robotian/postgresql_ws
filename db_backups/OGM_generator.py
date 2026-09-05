#!/usr/bin/env python3

import csv
import math
import argparse
import numpy as np
import os
import matplotlib.pyplot as plt

def world_to_grid(x, y, origin_x, origin_y, resolution, width_cells, height_cells):
    """Convert world coordinates (meters) to grid indices."""
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    
    # Constrain to map boundaries
    gx = max(0, min(gx, width_cells - 1))
    gy = max(0, min(gy, height_cells - 1))
    return gx, gy

def draw_circle(grid, cx, cy, radius, origin_x, origin_y, resolution):
    """Draw a circular obstacle on the grid."""
    h, w = grid.shape
    r_cells = int(radius / resolution)
    gx, gy = world_to_grid(cx, cy, origin_x, origin_y, resolution, w, h)
    
    for y in range(max(0, gy - r_cells), min(h, gy + r_cells + 1)):
        for x in range(max(0, gx - r_cells), min(w, gx + r_cells + 1)):
            if math.hypot(x - gx, y - gy) <= r_cells:
                grid[y, x] = 0  # 0 represents occupied (black)

def draw_rectangle(grid, cx, cy, width, height, origin_x, origin_y, resolution):
    """Draw a rectangular obstacle on the grid."""
    grid_h, grid_w = grid.shape
    
    min_x = cx - (width / 2.0)
    max_x = cx + (width / 2.0)
    min_y = cy - (height / 2.0)
    max_y = cy + (height / 2.0)
    
    g_min_x, g_min_y = world_to_grid(min_x, min_y, origin_x, origin_y, resolution, grid_w, grid_h)
    g_max_x, g_max_y = world_to_grid(max_x, max_y, origin_x, origin_y, resolution, grid_w, grid_h)
    
    for y in range(g_min_y, g_max_y + 1):
        for x in range(g_min_x, g_max_x + 1):
            grid[y, x] = 0  # 0 represents occupied (black)

def write_pgm(filename, grid):
    """Write the numpy array to a P5 PGM format image file."""
    # ROS maps expect (0,0) at the bottom-left, but image files have (0,0) at the top-left.
    flipped_grid = np.flipud(grid)
    height, width = flipped_grid.shape
    
    with open(filename, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(flipped_grid.tobytes())

def write_yaml(filename, pgm_filename, resolution, origin_x, origin_y):
    """Generate the Nav2 YAML metadata file."""
    yaml_content = f"""image: {pgm_filename}
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(filename, 'w') as f:
        f.write(yaml_content)

def display_map(grid, width, height, origin_x, origin_y):
    """Display the generated map using Matplotlib, ensuring the full map is visible."""
    extent = [origin_x, origin_x + width, origin_y, origin_y + height]
    
    # Calculate aspect ratio to prevent matplotlib from squishing/cropping the figure
    aspect_ratio = height / width
    fig_width = 8.0
    fig_height = fig_width * aspect_ratio
    
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    
    # Render image
    im = ax.imshow(grid, cmap='gray', vmin=0, vmax=255, origin='lower', extent=extent)
    
    # FORCED LIMITS: Prevents Matplotlib from zooming in automatically
    # ax.set_xlim(origin_x, origin_x + width)
    # ax.set_ylim(origin_y, origin_y + height)
    
    ax.set_title('Nav2 Occupancy Grid Map')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    
    # Add grid
    ax.grid(True, color='cyan', linestyle='--', linewidth=0.5, alpha=0.5)
    
    # TIGHT LAYOUT: Prevents edges/labels from being cut off by the window frame
    plt.tight_layout()
    
    print("Close the Matplotlib window to exit the script.")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Generate and display a ROS2 Nav2 Occupancy Grid Map.")
    parser.add_argument('--width', type=float, default=10.0, help='Map width in meters')
    parser.add_argument('--height', type=float, default=10.0, help='Map height in meters')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution (meters/cell)')
    parser.add_argument('--origin_x', type=float, default=-5.0, help='Map origin X (bottom-left corner)')
    parser.add_argument('--origin_y', type=float, default=-5.0, help='Map origin Y (bottom-left corner)')
    parser.add_argument('--csv_file', type=str, required=True, help='Path to input CSV file')
    parser.add_argument('--output_name', type=str, default='nav2_map', help='Base name for output files')
    
    args = parser.parse_args()

    width_cells = int(args.width / args.resolution)
    height_cells = int(args.height / args.resolution)
    
    grid = np.full((height_cells, width_cells), 255, dtype=np.uint8)
    
    if os.path.exists(args.csv_file):
        with open(args.csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                shape = row.get('shape', '').strip().lower()
                cx = float(row.get('x', 0.0))
                cy = float(row.get('y', 0.0))
                
                if shape == 'circle':
                    radius = float(row.get('size_1', 0.0))
                    draw_circle(grid, cx, cy, radius, args.origin_x, args.origin_y, args.resolution)
                elif shape == 'rectangle':
                    rect_width = float(row.get('size_1', 0.0))
                    rect_height = float(row.get('size_2', 0.0))
                    draw_rectangle(grid, cx, cy, rect_width, rect_height, args.origin_x, args.origin_y, args.resolution)
    else:
        print(f"Error: CSV file '{args.csv_file}' not found.")
        return

    pgm_file = f"{args.output_name}.pgm"
    yaml_file = f"{args.output_name}.yaml"
    
    write_pgm(pgm_file, grid)
    write_yaml(yaml_file, pgm_file, args.resolution, args.origin_x, args.origin_y)
    
    print(f"Map successfully generated:")
    print(f"  - Image: {pgm_file}")
    print(f"  - Config: {yaml_file}")

    display_map(grid, args.width, args.height, args.origin_x, args.origin_y)

if __name__ == '__main__':
    main()