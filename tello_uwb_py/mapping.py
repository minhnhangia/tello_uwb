import cv2
import numpy as np
import yaml
import math

from .constants import *

def export_to_ros_map(recorded_points, 
                      filename="uwb_map", 
                      resolution=0.05, 
                      padding_px=20, 
                      wall_thickness_m=1.0, 
                      pillar_radius_m=0.5):
    """
    Compiles UWB vector data into a ROS2-compatible Occupancy Grid.
    
    Args:
        recorded_points: List of point dictionaries from RecordingManager
        filename: Output filename (without extension)
        resolution: Meters per pixel (default 5cm)
        padding_px: Empty pixels to add around the recorded area
        wall_thickness_m: Thickness of walls in meters
        pillar_radius_m: Radius of pillars in meters
    """
    if not recorded_points:
        print("[WARN] No points to export!")
        return

    # 1. Filter out non-spatial points (like waypoints if you don't want them burned in)
    # Keeping them for now is fine, but usually we only map walls/pillars as obstacles.
    spatial_points = [p for p in recorded_points if 'x' in p and 'y' in p]
    
    # 2. Determine Bounds
    # We initialize bounds with 0,0 to ensure the origin is 'considered' 
    # if you want the map to always include the anchor. 
    # If you prefer tight cropping (even if 0,0 is off-map), remove the [0.0] lists.
    xs = [p['x'] for p in spatial_points] + [0.0] 
    ys = [p['y'] for p in spatial_points] + [0.0]
    
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    # 3. Calculate Canvas Size
    width_m = max_x - min_x
    height_m = max_y - min_y
    
    img_width = int(np.ceil(width_m / resolution)) + (padding_px * 2)
    img_height = int(np.ceil(height_m / resolution)) + (padding_px * 2)

    # Wall and Pillar thickness in pixels
    wall_thickness_px = int(np.ceil(wall_thickness_m / resolution))
    pillar_radius_px = int(np.ceil(pillar_radius_m / resolution))
    
    # 4. Initialize Map (254 = Free Space/White)
    # Standard ROS: 0=Occupied(Black), 254=Free(White), 205=Unknown(Gray)
    map_img = np.full((img_height, img_width), 254, dtype=np.uint8)
    
    # --- Coordinate Conversion Helper ---
    # Converts World (x,y) -> Image (col, row)
    def world_to_px(wx, wy):
        # Shift world coords by min_bounds to get positive offsets
        # Add padding
        px = int((wx - min_x) / resolution) + padding_px
        
        # Invert Y: Image origin is Top-Left, ROS World is Bottom-Left
        # We map World Y_min to Image Row_max
        py = img_height - (int((wy - min_y) / resolution) + padding_px) - 1
        return (px, py)
    
    # 5. Draw The "Vectors"
    wall_start_px = None
    
    for p in recorded_points:
        pt_px = world_to_px(p['x'], p['y'])
        
        if p.get('type') == 'wall_start':
            wall_start_px = pt_px
            # Draw the start node slightly
            cv2.circle(map_img, pt_px, wall_thickness_px // 2, (0,), -1)
            
        elif p.get('type') == 'wall_end' and wall_start_px:
            # Draw Wall (Black = 0)
            cv2.line(map_img, wall_start_px, pt_px, (0,), thickness=wall_thickness_px)
            # Draw end cap
            cv2.circle(map_img, pt_px, wall_thickness_px // 2, (0,), -1)
            wall_start_px = None
            
        elif p.get('type') == 'pillar':
            # Pillars are larger obstacles
            cv2.circle(map_img, pt_px, pillar_radius_px, (0,), thickness=-1)
                
    # 6. Draw Origin Marker (For visual verification)
    # This helps you confirm alignment. It draws a small crosshair at (0,0).
    origin_px = world_to_px(0.0, 0.0)
    # Check if origin is within image bounds before drawing
    if 0 <= origin_px[0] < img_width and 0 <= origin_px[1] < img_height:
        # Draw a light gray crosshair (200) so it's visible but not an obstacle
        cv2.line(map_img, (origin_px[0]-5, origin_px[1]), (origin_px[0]+5, origin_px[1]), (200,), 1)
        cv2.line(map_img, (origin_px[0], origin_px[1]-5), (origin_px[0], origin_px[1]+5), (200,), 1)

    # 7. Save PGM
    pgm_filename = f"{filename}.pgm"
    cv2.imwrite(str(UWBVIZ_DIR / pgm_filename), map_img)
    
    # 8. Save YAML
    # The 'origin' field is the World Coordinate of the Bottom-Left pixel.
    # Bottom-Left Pixel World X = min_x - (padding * res)
    # Bottom-Left Pixel World Y = min_y - (padding * res)
    origin_x = min_x - (padding_px * resolution)
    origin_y = min_y - (padding_px * resolution)
    
    yaml_data = {
        'image': pgm_filename,
        'resolution': resolution,
        'origin': [float(origin_x), float(origin_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    yaml_path = UWBVIZ_DIR / f"{filename}.yaml"
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f)
        
    print(f"[SUCCESS] Map exported to {yaml_path}")
    print(f"          Bounds: X[{min_x:.2f}, {max_x:.2f}] Y[{min_y:.2f}, {max_y:.2f}]")