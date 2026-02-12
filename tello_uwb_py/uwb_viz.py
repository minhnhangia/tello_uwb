#!/usr/bin/env python3

import pygame
import sys
import time
import threading
import json
import copy
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from tello_uwb_py.constants import *
from tello_uwb_py.utils import *
from tello_uwb_py.mapping import export_to_ros_map
import pandas as pd

from tello_uwb.msg import DronePositionArray

import tkinter as tk
from tkinter import simpledialog, ttk

"""
    ROS2 node for UWB position visualization
    Subscribes to /uwb/positions topic published by uwb_republisher
"""

# Create a Tkinter root window (but don't show it)
root = tk.Tk()
root.withdraw()  # Hide the main Tkinter window

user_input = ""  # Store the user's input

class UWBVisualization(Node):
    def __init__(self):
        super().__init__('uwb_visualization')
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("UWB Position Visualization")
        
        self.coord_system = CoordinateSystem()
        self.visualization = Visualization(self.screen, self.coord_system)
        self.background = Background(BGPIC)

        font = pygame.font.Font(None, 36)

        self.button1 = Button(100, 100, 200, 50, "Button 1", font, GREEN, BLUE, BLACK)
        self.button2 = Button(100, 200, 200, 50, "Button 2", font, GREEN, BLUE, BLACK)
        
        self.latest_positions_data = None       # Stores a local copy of ALL positions df
        self.data_lock = threading.Lock()
        self.controls_enabled = True
        self.panning = False
        self.last_mouse_pos = None
        self.waypoints = []
        self.loaded_marked_pos = None
        self.loaded_marked_pos_filename: str = None  

        self.tag_registered:int = 0

        self.last_cmdline: str = None        # Disabled 15 Feb - too complex to implement printing cmdline on pygame
        
        # Mouse simulation variables
        self.mouse_simulation = False
        self.simulated_tags = {
            0: {'x': 0, 'y': 0, 'z': 0},
        }
        
        # Recording manager
        self.recording_manager = RecordingManager()

        # Thread control for pause/resume during user input
        self.data_event = threading.Event()
        self.data_event.set()

        # ROS2 subscription to aggregated UWB positions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            DronePositionArray,
            '/uwb/positions',
            self.positions_callback,
            qos_profile
        )
        
        self.get_logger().info('UWB Visualization initialized')
        self.get_logger().info('Subscribed to /uwb/positions')

    def load_marked_positions(self, filename=None):
        """Load marked positions from JSON file. Do not include .json extension"""

        if filename:
            full_path = UWBVIZ_DIR / f"{filename}.json"
            print(f"Loading from: {full_path}")
        else:
            print("No filename provided. Not loading.")
            return

        try:
            with open(full_path, 'r') as f:
                self.loaded_marked_pos = json.load(f)["points"]
                self.loaded_marked_pos_filename = filename
            print(f"[INFO] Marked positions loaded successfully from {full_path}")
        except FileNotFoundError:
            print(f"[ERROR] File not found: {full_path}")
            print(f"[INFO] Make sure the file exists in {UWBVIZ_DIR}")
        except Exception as e:
            print(f"[ERROR] Could not load marked positions: {e}")

    def marked_points_dialog(self, parent_window=None):

        # Create a Tkinter window for the dropdown
        dialog_window = tk.Toplevel(parent_window)
        dialog_window.title("Marked Points Manager")
        dialog_width = 400
        dialog_height = 150  # Reduced height since we have fewer buttons
        dialog_window.geometry(f"{dialog_width}x{dialog_height}")
        
        # Force to be on top and take focus
        dialog_window.attributes("-topmost", True)
        
        # Center dialog on screen
        dialog_window.update_idletasks()
        screen_width = dialog_window.winfo_screenwidth()
        screen_height = dialog_window.winfo_screenheight()
        x = (screen_width - dialog_window.winfo_width()) // 2
        y = (screen_height - dialog_window.winfo_height()) // 2
        dialog_window.geometry(f"+{x}+{y}")
        
        # Get list of JSON files in UWBViz/ directory
        json_files = []
        try:
            # Ensure directory exists
            UWBVIZ_DIR.mkdir(parents=True, exist_ok=True)
            json_files = [f.replace('.json', '') for f in os.listdir(UWBVIZ_DIR) if f.endswith('.json')]
        except FileNotFoundError:
            print(f"Warning: Directory {UWBVIZ_DIR} not found")
        except Exception as e:
            print(f"Warning: Could not list files in {UWBVIZ_DIR}: {e}")
        
        # Ensure default is in the list
        if MARKEDPOINTS_JSON_DEFAULT not in json_files:
            json_files.insert(0, MARKEDPOINTS_JSON_DEFAULT)
        else:
            # Move default to the first position
            json_files.remove(MARKEDPOINTS_JSON_DEFAULT)
            json_files.insert(0, MARKEDPOINTS_JSON_DEFAULT)
        
        # Create label
        tk.Label(dialog_window, text=f"Select JSON file to load:\nLast loaded: {self.loaded_marked_pos_filename}").pack(pady=10)
        
        # Create combobox (dropdown)
        filename_var = tk.StringVar(value=MARKEDPOINTS_JSON_DEFAULT)
        filename_combo = ttk.Combobox(dialog_window, textvariable=filename_var, values=json_files, width=30)
        filename_combo.pack(pady=5)
        filename_combo.current(0)  # Select default option
        
        # Make combobox read-only for load
        filename_combo.configure(state="readonly")
        
        # Variables to store result
        result = {"filename": None, "action": None}
        
        def on_load():
            result["filename"] = filename_var.get()
            result["action"] = "load"
            dialog_window.destroy()
        
        def on_cancel():
            dialog_window.destroy()

        def on_clear():
            """
            Resets the marked points to None
            """
            self.loaded_marked_pos = None
            self.recording_manager.recorded_points = []
            self.recording_manager.current_wall_id = 0
            self.loaded_marked_pos_filename = None
            dialog_window.destroy()
        
        # Handle Enter key press to load
        def on_enter(event):
            on_load()
        
        # Handle Escape key press to cancel
        def on_escape(event):
            on_cancel()
        
        # Bind Enter and Escape keys to the window
        dialog_window.bind('<Return>', on_enter)
        dialog_window.bind('<Escape>', on_escape)
        
        # Button frame
        button_frame = tk.Frame(dialog_window)
        button_frame.pack(pady=15)
        
        # Load and Cancel buttons
        load_button = tk.Button(button_frame, text="Load", command=on_load)
        cancel_button = tk.Button(button_frame, text="Cancel", command=on_cancel)
        clear_button = tk.Button(button_frame, text="Clear", command=on_clear)
        
        # Load button has initial focus
        load_button.config(relief=tk.SUNKEN)
        
        # Pack buttons
        load_button.pack(side=tk.LEFT, padx=10)
        cancel_button.pack(side=tk.LEFT, padx=10)
        clear_button.pack(side=tk.LEFT, padx=10)
        
        # Ensure the window takes focus
        dialog_window.after(100, lambda: dialog_window.focus_force())
        filename_combo.focus_set()
        
        # Make window modal (blocks until closed)
        dialog_window.grab_set()
        dialog_window.wait_window()
        
        # Process result
        if result["action"] == "load":
            # Call load_marked_positions function with the selected filename
            self.load_marked_positions(result["filename"])
            print(f"Loaded marked points from {result['filename']}.json")
            return True
        
        return False

    def export_map_dialog(self, parent_window=None):
        """Dialog to select a JSON file from UWBVIZ_DIR and export it as a ROS2 occupancy grid map."""

        # Create a Tkinter window for the dialog
        dialog_window = tk.Toplevel(parent_window)
        dialog_window.title("Export Occupancy Grid Map")
        dialog_width = 450
        dialog_height = 340
        dialog_window.geometry(f"{dialog_width}x{dialog_height}")

        # Force to be on top and take focus
        dialog_window.attributes("-topmost", True)

        # Center dialog on screen
        dialog_window.update_idletasks()
        screen_width = dialog_window.winfo_screenwidth()
        screen_height = dialog_window.winfo_screenheight()
        x = (screen_width - dialog_window.winfo_width()) // 2
        y = (screen_height - dialog_window.winfo_height()) // 2
        dialog_window.geometry(f"+{x}+{y}")

        # Get list of JSON files in UWBViz/ directory
        json_files = []
        try:
            UWBVIZ_DIR.mkdir(parents=True, exist_ok=True)
            json_files = sorted([f.replace('.json', '') for f in os.listdir(UWBVIZ_DIR) if f.endswith('.json')])
        except Exception as e:
            print(f"Warning: Could not list files in {UWBVIZ_DIR}: {e}")

        if not json_files:
            print("[WARN] No JSON files found in UWBViz directory.")
            dialog_window.destroy()
            return False

        # --- Source file selector ---
        tk.Label(dialog_window, text="Source JSON (walls/pillars):").pack(pady=(10, 2))
        source_var = tk.StringVar(value=json_files[0])
        source_combo = ttk.Combobox(dialog_window, textvariable=source_var, values=json_files, width=35)
        source_combo.pack(pady=2)
        source_combo.configure(state="readonly")

        # Pre-select the currently loaded file if available
        if self.loaded_marked_pos_filename and self.loaded_marked_pos_filename in json_files:
            source_combo.set(self.loaded_marked_pos_filename)

        # --- Output filename ---
        tk.Label(dialog_window, text="Output filename (without extension):").pack(pady=(8, 2))
        output_var = tk.StringVar(value="")
        output_entry = tk.Entry(dialog_window, textvariable=output_var, width=38)
        output_entry.pack(pady=2)

        # --- Resolution ---
        res_frame = tk.Frame(dialog_window)
        res_frame.pack(pady=(8, 2))
        tk.Label(res_frame, text="Resolution (m/px):").pack(side=tk.LEFT, padx=(0, 5))
        res_var = tk.StringVar(value="0.05")
        res_entry = tk.Entry(res_frame, textvariable=res_var, width=8)
        res_entry.pack(side=tk.LEFT)

        # --- Safety Buffer: Wall Thickness ---
        wall_frame = tk.Frame(dialog_window)
        wall_frame.pack(pady=(8, 2))
        tk.Label(wall_frame, text="Wall Thickness (m):").pack(side=tk.LEFT, padx=(0, 5))
        wall_var = tk.StringVar(value="1.0")
        wall_entry = tk.Entry(wall_frame, textvariable=wall_var, width=8)
        wall_entry.pack(side=tk.LEFT)

        # --- Safety Buffer: Pillar Radius ---
        pillar_frame = tk.Frame(dialog_window)
        pillar_frame.pack(pady=(2, 2))
        tk.Label(pillar_frame, text="Pillar Radius (m):").pack(side=tk.LEFT, padx=(0, 5))
        pillar_var = tk.StringVar(value="0.5")
        pillar_entry = tk.Entry(pillar_frame, textvariable=pillar_var, width=8)
        pillar_entry.pack(side=tk.LEFT)

        result = {"action": None}

        def on_export():
            result["action"] = "export"
            dialog_window.destroy()

        def on_cancel():
            dialog_window.destroy()

        dialog_window.bind('<Return>', lambda e: on_export())
        dialog_window.bind('<Escape>', lambda e: on_cancel())

        # Button frame
        button_frame = tk.Frame(dialog_window)
        button_frame.pack(pady=12)
        tk.Button(button_frame, text="Export", command=on_export).pack(side=tk.LEFT, padx=10)
        tk.Button(button_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=10)

        dialog_window.after(100, lambda: dialog_window.focus_force())
        source_combo.focus_set()

        # Make window modal
        dialog_window.grab_set()
        dialog_window.wait_window()

        # --- Process result ---
        if result["action"] != "export":
            return False

        source_name = source_var.get()
        output_name = output_var.get().strip() or source_name

        # Parse resolution
        try:
            resolution = float(res_var.get())
            if resolution <= 0:
                raise ValueError
        except ValueError:
            print("[ERROR] Invalid resolution value. Using default 0.05 m/px.")
            resolution = 0.05

        # Parse wall thickness
        try:
            wall_thickness = float(wall_var.get())
            if wall_thickness < 0:
                raise ValueError
        except ValueError:
            print("[ERROR] Invalid wall thickness value. Using default 1.0 m.")
            wall_thickness = 1.0

        # Parse pillar radius
        try:
            pillar_radius = float(pillar_var.get())
            if pillar_radius < 0:
                raise ValueError
        except ValueError:
            print("[ERROR] Invalid pillar radius value. Using default 0.5 m.")
            pillar_radius = 0.5

        # Load points from selected JSON
        source_path = UWBVIZ_DIR / f"{source_name}.json"
        try:
            with open(source_path, 'r') as f:
                data = json.load(f)
            points = data.get("points", [])
        except Exception as e:
            print(f"[ERROR] Could not load {source_path}: {e}")
            return False

        if not points:
            print(f"[WARN] No points found in {source_name}.json")
            return False

        # Call the export function with safety buffer parameters
        export_to_ros_map(points, 
                         filename=output_name, 
                         resolution=resolution,
                         wall_thickness_m=wall_thickness,
                         pillar_radius_m=pillar_radius)
        return True

    def positions_callback(self, msg: DronePositionArray):
        """Callback for receiving aggregated UWB positions from ROS2 topic."""
        # Don't process real data when in mouse simulation mode
        if self.mouse_simulation:
            return
        
        # Don't process data when paused (e.g., during user input dialogs)
        if not self.data_event.is_set():
            return
        
        # Convert ROS2 message to DataFrame format expected by visualization
        rows = []
        for drone in msg.drones:
            rows.append({
                'id': drone.uwb_tag_id,  # Use numeric UWB tag ID
                'role': 2,  # Assume role 2 (tags) - already filtered by uwb_republisher
                'x': drone.position.point.x,
                'y': drone.position.point.y,
                'z': drone.position.point.z,
                'timestamp': drone.position.header.stamp.sec + 
                            drone.position.header.stamp.nanosec * 1e-9
            })
        
        if rows:
            df = pd.DataFrame(rows)
            with self.data_lock:
                self.latest_positions_data = df
    
    def update_mouse_simulation(self):
        """Update simulated tag position based on mouse position."""
        if not self.mouse_simulation:
            return
        
        mouse_x, mouse_y = pygame.mouse.get_pos()
        uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
        self.simulated_tags[0]['x'] = uwb_x
        self.simulated_tags[0]['y'] = uwb_y
        
        # Convert simulated data to DataFrame
        rows = []
        for tag_id, pos in self.simulated_tags.items():
            rows.append({
                'id': tag_id,
                'role': 0,
                'x': pos['x'],
                'y': pos['y'],
                'z': pos['z'],
                'timestamp': time.time()
            })
        df = pd.DataFrame(rows)
        
        with self.data_lock:
            self.latest_positions_data = df

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and 
                event.key == pygame.K_ESCAPE
            ):
                if self.recording_manager.recording:
                    filename = self.recording_manager.stop_recording(self.loaded_marked_pos)
                    self.load_marked_positions(filename)
                return False
                    
            elif event.type == pygame.VIDEORESIZE:
                # Handle window resize
                self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                self.visualization.screen = self.screen  # Update visualization's screen reference
                    
            elif event.type == pygame.MOUSEWHEEL and self.controls_enabled:
                self.handle_zoom(event.y)
                    
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.controls_enabled:
                    self.panning = True
                    self.last_mouse_pos = pygame.mouse.get_pos()

                # Check for button clicks
                mouse_pos = pygame.mouse.get_pos()
                if self.button1.is_clicked(mouse_pos):
                    print("Button 1 clicked!")
                    # Trigger your event here
                if self.button2.is_clicked(mouse_pos):
                    print("Button 2 clicked!")
                    # Trigger your event here
                        
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.panning = False
                    
            elif event.type == pygame.MOUSEMOTION and self.panning and self.controls_enabled:
                current_pos = pygame.mouse.get_pos()
                self.coord_system.offset_x += current_pos[0] - self.last_mouse_pos[0]
                self.coord_system.offset_y += current_pos[1] - self.last_mouse_pos[1]
                self.last_mouse_pos = current_pos
                    
            elif event.type == pygame.KEYDOWN:
                self.handle_keypress(event.key)

        # Update hover state for buttons
        mouse_pos = pygame.mouse.get_pos()
        self.button1.check_hover(mouse_pos)
        self.button2.check_hover(mouse_pos)

        return True

    def handle_zoom(self, y):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        pre_zoom_uwb_x, pre_zoom_uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
        old_scale = self.coord_system.scale_factor
        self.coord_system.scale_factor = max(10, min(INITIAL_SCALE * 10, self.coord_system.scale_factor + y * 5))
        new_screen_x, new_screen_y = self.coord_system.screen_coordinates(pre_zoom_uwb_x, pre_zoom_uwb_y)
        self.coord_system.offset_x += (mouse_x - new_screen_x)
        self.coord_system.offset_y += (mouse_y - new_screen_y)

    def handle_keypress(self, key):
        current_pos = None
        if self.mouse_simulation:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
            current_pos = {'x': uwb_x, 'y': uwb_y, 'z': 0}
        elif self.latest_positions_data is not None and not self.latest_positions_data.empty:
            # print(f"DEBUG 11 MAR: latest_positions_data = {self.latest_positions_data}")
            if self.tag_registered != 0:
                last_pos = self.latest_positions_data.loc[self.latest_positions_data['id'] == self.tag_registered]
            else:
                last_pos = self.latest_positions_data.iloc[-1]      # caa 11 Mar : currently takes the LARGEST tag number (to work)
            
            current_pos = {'x': last_pos['x'], 'y': last_pos['y'], 'z': last_pos.get('z', 0)}

        if key == pygame.K_SPACE:
            self.visualization.persistent_trails = not self.visualization.persistent_trails
            if not self.visualization.persistent_trails:
                self.visualization.position_history.clear()
        elif key == pygame.K_RETURN:
            self.controls_enabled = not self.controls_enabled
            if not self.controls_enabled:
                self.panning = False
        elif key == pygame.K_l:
            self.pause_data_thread()
            self.waypoints = load_waypoints()
            self.resume_data_thread()
        elif key == pygame.K_u:  # Toggle mouse simulation
            self.mouse_simulation = not self.mouse_simulation
            if self.mouse_simulation:
                print("[INFO] Mouse simulation enabled")
            else:
                print("[INFO] Mouse simulation disabled")
        elif key == pygame.K_r:  # Toggle recording
            if not self.recording_manager.recording:
                self.recording_manager.start_recording()
            else:
                filename = self.recording_manager.stop_recording(self.loaded_marked_pos)
                self.load_marked_positions(filename)
        elif key == pygame.K_w and current_pos:  # Toggle wall recording
            self.recording_manager.toggle_wall(current_pos['x'], current_pos['y'], current_pos['z'])
        elif key == pygame.K_p and current_pos:  # Add pillar obstacle
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'pillar')
        elif key == pygame.K_v and current_pos:
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'victim')
        elif key == pygame.K_d and current_pos:
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'danger')
        elif key == pygame.K_q and current_pos and self.recording_manager.recording:    # Add waypoint
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'waypoint')
        elif key == pygame.K_z:
            self.recording_manager.remove_last_obj()
        elif key == pygame.K_m:
            result = self.marked_points_dialog()
            if result:
                print("Marked points loaded successfully")
            else:
                print("Marked points not loaded")
        elif key == pygame.K_e and not self.recording_manager.recording:
            self.pause_data_thread()
            self.export_map_dialog()
            self.resume_data_thread()
        elif key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9, pygame.K_0) and not self.recording_manager.recording:
            number_pressed = key_to_number.get(key)
            self.tag_registered = int(number_pressed)
            print(f"Registering tag number: {number_pressed}")

    def pause_data_thread(self):
        """Pause processing of ROS2 position updates during user input."""
        self.get_logger().info('Pausing position updates for user input')
        self.data_event.clear()

    def resume_data_thread(self):
        """Resume processing of ROS2 position updates."""
        self.get_logger().info('Resuming position updates')
        self.data_event.set()

    def update(self):
        # Update mouse simulation if enabled
        if self.mouse_simulation:
            self.update_mouse_simulation()
        
        with self.data_lock:
            current_positions = copy.deepcopy(self.latest_positions_data)
        
        if current_positions is not None:
            self.visualization.update_positions(current_positions)

    def draw(self):
        self.screen.fill(BACKGROUND_COLOR)
        
        # Get current screen dimensions
        screen_width, screen_height = self.screen.get_size()
        
        self.background.draw(self.screen, self.coord_system.scale_factor, self.coord_system.screen_coordinates)
        self.visualization.draw_grid()
        self.visualization.draw_rectangle()
        self.visualization.draw_positions()  # Draw UWB positions

        self.visualization.draw_loaded_waypoints(self.waypoints)  # Draw pre-loaded waypoints
        
        if self.loaded_marked_pos:  # Draws pre-loaded positions
            self.visualization.draw_marked_positions(self.loaded_marked_pos)

        if self.recording_manager.recorded_points:  # Draw marked positions and newly marked waypoints
            self.visualization.draw_marked_positions(self.recording_manager.recorded_points)

        # === UI OVERLAY ===
        
        # Recording status banner (top)
        if self.recording_manager.recording:
            font_status = pygame.font.Font(None, 40)
            if self.recording_manager.recording_wall:
                status = f"🔴 RECORDING WALL #{self.recording_manager.current_wall_id}"
                status_color = (255, 50, 50)  # Bright red
            else:
                status = "🔴 RECORDING MODE - Press R to Stop"
                status_color = (255, 100, 0)  # Orange
            
            text = font_status.render(status, True, status_color)
            text_width, text_height = font_status.size(status)
            
            # Semi-transparent background
            s = pygame.Surface((text_width + 30, text_height + 20))
            s.set_alpha(220)
            s.fill((255, 255, 255))
            self.screen.blit(s, (10, 5))
            self.screen.blit(text, (25, 12))

        # Status Panel (top-right)
        y_offset = 10
        font_status_small = pygame.font.Font(None, 24)
        status_lines = []
        
        # Selected tag
        tag_text = f"Tag: {self.tag_registered if self.tag_registered != 0 else 'Auto (largest)'}"
        status_lines.append((tag_text, (0, 100, 200)))
        
        # Mouse simulation
        if self.mouse_simulation:
            status_lines.append(("🖱️  Mouse Simulation ON", (200, 0, 200)))
        
        # Pan/Zoom
        pan_status = "Pan/Zoom: ON" if self.controls_enabled else "Pan/Zoom: LOCKED"
        status_lines.append((pan_status, (100, 100, 100)))
        
        # Trails
        trail_status = "Trails: Persistent" if self.visualization.persistent_trails else "Trails: Fading"
        status_lines.append((trail_status, (100, 100, 100)))
        
        # Draw status panel
        max_width = max([font_status_small.size(line[0])[0] for line in status_lines])
        panel_height = len(status_lines) * 25 + 10
        s = pygame.Surface((max_width + 20, panel_height))
        s.set_alpha(200)
        s.fill((255, 255, 255))
        self.screen.blit(s, (screen_width - max_width - 30, y_offset))
        
        for i, (text_str, color) in enumerate(status_lines):
            text = font_status_small.render(text_str, True, color)
            self.screen.blit(text, (screen_width - max_width - 20, y_offset + 5 + i * 25))

        # Instructions Panel (bottom-left)
        font_section = pygame.font.Font(None, 26)
        font_key = pygame.font.Font(None, 22)
        
        instructions = []
        
        if self.recording_manager.recording:
            # Recording mode instructions
            instructions.extend([
                ("RECORDING CONTROLS:", (255, 50, 0), True),
                ("Q: Add Waypoint", (0, 0, 0), False),
                ("W: Toggle Wall Recording", (0, 0, 0), False),
                ("P: Mark Pillar", (0, 0, 0), False),
                ("V: Mark Victim", (0, 0, 0), False),
                ("D: Mark Danger Zone", (0, 0, 0), False),
                ("Z: Undo Last Mark", (0, 0, 0), False),
                ("R: Stop Recording & Save", (255, 0, 0), False),
                ("", (0, 0, 0), False),
            ])
        else:
            # Normal mode instructions
            instructions.extend([
                ("CONTROLS:", (0, 100, 200), True),
                ("R: Start Recording", (0, 0, 0), False),
                ("M: Load Markers", (0, 0, 0), False),
                ("L: Load Waypoints", (0, 0, 0), False),
                ("E: Export Occupancy Grid", (0, 0, 0), False),
                ("U: Toggle Mouse Simulation", (0, 0, 0), False),
                ("1-9/0: Select Tag for Recording", (0, 0, 0), False),
                ("", (0, 0, 0), False),
            ])
        
        instructions.extend([
            ("VIEW:", (0, 100, 200), True),
            ("SPACE: Toggle Trails", (0, 0, 0), False),
            ("ENTER: Toggle Pan/Zoom Lock", (0, 0, 0), False),
            ("Scroll: Zoom In/Out", (0, 0, 0), False),
            ("Left-Click Drag: Pan View", (0, 0, 0), False),
            ("", (0, 0, 0), False),
            ("ESC: Exit", (150, 0, 0), False),
        ])
        
        # Calculate panel size
        max_instr_width = 0
        for text_str, _, is_section in instructions:
            if text_str:
                font_to_use = font_section if is_section else font_key
                width = font_to_use.size(text_str)[0]
                max_instr_width = max(max_instr_width, width)
        
        panel_height = len(instructions) * 22 + 15
        s = pygame.Surface((max_instr_width + 30, panel_height))
        s.set_alpha(200)
        s.fill((255, 255, 255))
        self.screen.blit(s, (10, screen_height - panel_height - 10))
        
        # Draw instructions
        y_pos = screen_height - panel_height - 5
        for text_str, color, is_section in instructions:
            if text_str:
                font_to_use = font_section if is_section else font_key
                text = font_to_use.render(text_str, True, color)
                self.screen.blit(text, (20, y_pos))
            y_pos += 22

        pygame.display.update()

    def run(self):
        clock = pygame.time.Clock()
        self.get_logger().info('Starting visualization loop')
        self.get_logger().info('Controls: ESC=exit, SPACE=trails, ENTER=pan/zoom, L=load waypoints')
        self.get_logger().info('Recording: R=toggle, W=walls, V/D/P=victims/dangers/pillars, M=load markers')
        
        running = True
        while running and rclpy.ok():
            # Process ROS2 callbacks (non-blocking)
            rclpy.spin_once(self, timeout_sec=0)
            
            # Process Pygame events and rendering
            running = self.handle_events()
            self.update()
            self.draw()
            clock.tick(30)
        
        self.get_logger().info('Shutting down visualization')
        pygame.quit()

def main(args=None):
    """Main entry point for ROS2 node."""
    rclpy.init(args=args)
    
    app = None
    try:
        app = UWBVisualization()
        app.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error during execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if app is not None and rclpy.ok():
            app.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()