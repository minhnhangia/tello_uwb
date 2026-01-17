# constants.py
import pygame
import os
from pathlib import Path

# Display settings
SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 800
POINT_RADIUS = 7
FONT_SIZE = 30
INITIAL_SCALE = 40
MAX_HISTORY = 50
TRACE_FADE_TIME = 10
RECT_WIDTH, RECT_HEIGHT = 20, 20        # IMPT: Scale in metres

# Colors
BACKGROUND_COLOR = (255, 255, 255)  # White
GRID_COLOR = (200, 200, 200)       # Light gray
LABEL_COLOR = (100, 100, 100)      # Dark gray
BLUE = (0, 0, 255)
RED = (255, 0, 0)

# File paths - Use absolute paths from workspace root
WORKSPACE_ROOT = Path.home() / "tello_ros_ws" / "src" / "tello_localization" / "tello_uwb"
RESOURCES_DIR = WORKSPACE_ROOT / "resources"
BGPIC = str(RESOURCES_DIR / "field2025_toscale.png")

# Data directories
DATA_DIR = WORKSPACE_ROOT / "data"
UWBVIZ_DIR = DATA_DIR / "UWBViz"
WAYPOINTS_DIR = DATA_DIR / "waypoints"

# Default filenames (without extensions)
WAYPOINTS_JSON_DEFAULT = "waypoint20x20"
MARKEDPOINTS_JSON_DEFAULT = "uwb_trace"
MARKEDWAYPOINTS_JSON_DEFAULT = "waypoints_uwb"

# Ensure directories exist
for directory in [DATA_DIR, UWBVIZ_DIR, WAYPOINTS_DIR, RESOURCES_DIR]:
    directory.mkdir(parents=True, exist_ok=True)

# Tag colors for different UWB tags
TAG_COLORS = [
    (0, 51, 102),   # Navy Blue
    (153, 0, 0),    # Dark Red
    (0, 102, 51),   # Forest Green
    (102, 0, 102),  # Purple
    (153, 76, 0),   # Dark Orange
    (51, 51, 0),    # Olive
    (102, 51, 0),   # Brown
    (51, 0, 51),    # Dark Purple
    (0, 76, 153),   # Royal Blue
    (153, 0, 76),   # Dark Pink
]

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)