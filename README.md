# UWBViz - README

**Last Updated:** 17 Jan 2026

## Overview
ROS2 Jazzy GUI node to display live UWB tag positions in the arena. Subscribes to `/uwb/positions` topic published by `uwb_republisher` node. Tag data can be used to record obstacles, waypoints, etc., or simply track the motion of drones.

## ROS2 Architecture

### Node: `uwb_visualization`
- **Type:** Python ROS2 node with Pygame GUI
- **Subscription:** `/uwb/positions` (tello_uwb/DronePositionArray)
- **QoS:** Best Effort, Volatile (matches sensor data profile)

### Data Flow
```
LinkTrack UWB → uwb_republisher (C++) → /uwb/positions → uwb_viz.py (GUI)
```

## Running the Node

### Prerequisites
1. Build the package:
   ```bash
   cd ~/tello_ros_ws
   colcon build --packages-select tello_uwb
   source install/setup.bash
   ```

2. Ensure `uwb_republisher` node is running:
   ```bash
   ros2 run tello_uwb uwb_republisher
   # Or use launch file with configuration
   ```

### Start Visualization
```bash
ros2 run tello_uwb uwb_viz.py
```

## Usage

### Controls
- **R** - Start/Stop recording. Upon stopping recording, prompts user to save:
    1. Marked points (all obstacles + waypoints)
    2. Waypoints separately (compatible with mission planner)
- **U** - Toggle simulated tag #0 using mouse position
- **Space** - Toggle UWB tag shadow tracing
- **Enter** - Toggle pan lock (enable/disable pan & zoom)
- **M** - Load marked points (obstacles and/or waypoints) from JSON file in UWBViz/ directory
- **L** - Load waypoints from JSON file in parent directory
- **1~0** - Toggle UWB tag data to be used for recording. By default, records largest tag number available.
- **Scroll Wheel** - Zoom in/out
- **Left Click + Drag** - Pan view (when pan enabled)

### Recording Mode (press R to start)
- **Q** - Record waypoint at current tag position
- **Z** - Undo last marked item
- **W** - Toggle wall recording (start/stop pair)
- **P** - Mark pillar obstacle
- **V** - Mark victim location
- **D** - Mark danger zone

## Features

### Real-time Visualization
- Multi-tag position tracking with unique colors
- Persistent trail visualization
- Coordinate grid with labels
- Background field image (20m x 20m arena)
- Pan and zoom controls

### Recording System
- **Waypoints:** Sequential navigation points
- **Walls:** Start/end pairs forming barriers
- **Obstacles:** Pillars, victims, danger zones
- **Undo:** Remove last recorded object
- **Tag Selection:** Choose which UWB tag to track for recording

### Data Management
- **Save:** Exports to JSON with metadata (counts, timestamps)
- **Load:** Dialog to select/load existing marked positions
- **Merge:** Option to combine loaded + new recordings
- **Dual Export:** 
  - Marked positions JSON (all objects)
  - Waypoints JSON (converted format for mission planner)

### Mouse Simulation Mode
- Press **U** to enable/disable
- Simulates tag #0 at mouse cursor position
- Useful for testing without hardware

## Message Format

### DronePositionArray
```
DronePosition[] drones
```

### DronePosition
```
string drone_id        # Drone name (e.g., "tello1")
int32 uwb_tag_id      # Numeric UWB tag ID (0-9)
geometry_msgs/PointStamped position
```

## Configuration

The visualization automatically receives configured drones from `uwb_republisher`. No additional configuration needed for basic usage.

### Arena Settings (constants.py)
- `SCREEN_WIDTH, SCREEN_HEIGHT`: Window size (default: 1000x800)
- `RECT_WIDTH, RECT_HEIGHT`: Arena size in meters (default: 20x20)
- `INITIAL_SCALE`: Zoom factor (default: 40 pixels/meter)
- `BGPIC`: Background image path

## File Outputs

### Marked Positions JSON
```json
{
  "points": [
    {"x": 1.5, "y": 2.3, "z": 0.0, "type": "waypoint"},
    {"x": 0.0, "y": 0.0, "z": 0.0, "type": "wall_start", "wall_id": 1},
    {"x": 5.0, "y": 0.0, "z": 0.0, "type": "wall_end", "wall_id": 1}
  ],
  "metadata": {
    "total_points": 3,
    "total_wall_segments": 1,
    "total_waypoints": 1
  }
}
```

### Waypoints JSON (Mission Planner Format)
```json
{
  "README": "Each waypoint stores distance/angle to NEXT waypoint",
  "wp": [
    {
      "dist_cm": 250,
      "angle_deg": 45,
      "position_cm": {"x": 150, "y": 230}
    }
  ],
  "datetime": "01 17 14:30"
}
```

## Technical Details

### Threading Model
- **Main Thread:** Pygame event loop + ROS2 spin_once (non-blocking)
- **Data Synchronization:** Threading.Lock protects position DataFrame
- **Rate:** 30 FPS rendering

### Coordinate Systems
- **UWB:** Real-world meters (origin at field corner)
- **Screen:** Pixels with configurable scale factor
- **Transformations:** Handles pan offset and zoom

### ROS2 Integration
- Uses `rclpy.spin_once(timeout_sec=0)` for non-blocking callback processing
- Interleaved with Pygame event handling at 30 Hz
- Proper cleanup on node shutdown

## Dependencies
- ROS2 Jazzy
- Python packages:
  - rclpy
  - pygame
  - pandas
  - tkinter (for dialogs)
- ROS2 packages:
  - tello_uwb (this package)
  - nlink_parser (for LinkTrack messages)

## Troubleshooting

### No data displayed
1. Check if `uwb_republisher` is running: `ros2 node list`
2. Verify topic is publishing: `ros2 topic echo /uwb/positions`
3. Check ROS2 network configuration (DDS settings)

### Build errors
```bash
# Clean rebuild
cd ~/tello_ros_ws
rm -rf build/tello_uwb install/tello_uwb
colcon build --packages-select tello_uwb
```

### Import errors
```bash
# Ensure workspace is sourced
source ~/tello_ros_ws/install/setup.bash
```

## Migration Notes (from Non-ROS2 Version)

### Key Changes
1. **Data Source:** UDP (`get_all_positions()`) → ROS2 topic (`/uwb/positions`)
2. **Architecture:** Standalone thread → ROS2 Node with integrated spinning
3. **Message Format:** Custom DataFrame → ROS2 DronePositionArray message
4. **Tag IDs:** Direct numeric ID from `uwb_tag_id` field
5. **Entry Point:** Direct execution → ROS2 node with proper init/shutdown

### Preserved Features
- All keyboard shortcuts and controls
- Recording system functionality
- Coordinate transformation logic
- File save/load operations
- Mouse simulation mode
- Tkinter dialogs (paused during user input)

## Additional Notes

- **Network:** Works on any ROS2-compatible network (unlike UDP-only previous version)
- **Scalability:** Handles multiple drones automatically via `uwb_republisher` configuration
- **Performance:** Non-blocking ROS2 integration maintains 30 FPS rendering