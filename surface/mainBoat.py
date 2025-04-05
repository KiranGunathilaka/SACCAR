import socket
import cv2
import threading
import struct
import numpy as np
import time
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cmd
import sys

# Global variables
latest_frame = None
latest_status = None
latest_mcu_data = None
frame_count = 0
stop_event = threading.Event()
conn_lock = threading.Lock()
client_connection = None

class StatusDisplay:
    """Separate window for displaying telemetry and status information"""
    def __init__(self, root):
        self.root = root
        self.root.title("Submersible Status")
        self.root.geometry("500x600")
        
        # Create tabs
        self.tab_control = ttk.Notebook(root)
        
        # Main Status Tab
        self.status_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.status_tab, text='Main Status')
        
        # MCU Data Tab
        self.mcu_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.mcu_tab, text='MCU Data')
        
        self.tab_control.pack(expand=1, fill="both")
        
        # Status variables
        self.depth_var = tk.StringVar(value="Depth: 0.0 m")
        self.temp_var = tk.StringVar(value="Temperature: 0.0 °C")
        self.battery_var = tk.StringVar(value="Battery: 0.0%")
        self.pitch_var = tk.StringVar(value="Pitch: 0.0°")
        self.roll_var = tk.StringVar(value="Roll: 0.0°")
        self.yaw_var = tk.StringVar(value="Yaw: 0.0°")
        self.water_var = tk.StringVar(value="Water: Not Detected")
        
        # MCU variables
        self.pressure_var = tk.StringVar(value="Pressure: 0.0 hPa")
        self.internal_temp_var = tk.StringVar(value="Internal Temp: 0.0 °C")
        self.humidity_var = tk.StringVar(value="Humidity: 0.0%")
        self.leak_var = tk.StringVar(value="Leak Sensor: 0.0 V")
        self.motor1_var = tk.StringVar(value="Motor 1 Current: 0.0 A")
        self.motor2_var = tk.StringVar(value="Motor 2 Current: 0.0 A")
        self.motor3_var = tk.StringVar(value="Motor 3 Current: 0.0 A")
        self.motor4_var = tk.StringVar(value="Motor 4 Current: 0.0 A")
        
        # Status layout
        self._create_status_layout()
        
        # MCU data layout
        self._create_mcu_layout()
        
        # Control panel
        self._create_control_panel()
        
        # Set update interval
        self.root.after(100, self.update_status)
    
    def _create_status_layout(self):
        """Create the status tab layout"""
        # Title
        title_label = ttk.Label(self.status_tab, text="Submersible Status", font=("Arial", 16))
        title_label.pack(pady=10)
        
        # Main readings frame
        readings_frame = ttk.LabelFrame(self.status_tab, text="Readings")
        readings_frame.pack(fill="x", padx=10, pady=5)
        
        # Main readings
        ttk.Label(readings_frame, textvariable=self.depth_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(readings_frame, textvariable=self.temp_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(readings_frame, textvariable=self.battery_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        
        # Orientation frame
        orientation_frame = ttk.LabelFrame(self.status_tab, text="Orientation")
        orientation_frame.pack(fill="x", padx=10, pady=5)
        
        # Orientation readings
        ttk.Label(orientation_frame, textvariable=self.pitch_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(orientation_frame, textvariable=self.roll_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(orientation_frame, textvariable=self.yaw_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        
        # Warnings frame
        warnings_frame = ttk.LabelFrame(self.status_tab, text="Warnings")
        warnings_frame.pack(fill="x", padx=10, pady=5)
        
        # Warning indicators
        self.water_label = ttk.Label(warnings_frame, textvariable=self.water_var, font=("Arial", 12))
        self.water_label.pack(anchor="w", padx=5, pady=2)
        
        # Canvas for artificial horizon
        self.horizon_canvas = tk.Canvas(self.status_tab, width=200, height=200, bg="black")
        self.horizon_canvas.pack(pady=10)
    
    def _create_mcu_layout(self):
        """Create the MCU data tab layout"""
        # Title
        title_label = ttk.Label(self.mcu_tab, text="MCU Sensor Data", font=("Arial", 16))
        title_label.pack(pady=10)
        
        # Environmental readings frame
        env_frame = ttk.LabelFrame(self.mcu_tab, text="Environmental")
        env_frame.pack(fill="x", padx=10, pady=5)
        
        # Environmental readings
        ttk.Label(env_frame, textvariable=self.pressure_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(env_frame, textvariable=self.internal_temp_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(env_frame, textvariable=self.humidity_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        
        # Leak sensor frame
        leak_frame = ttk.LabelFrame(self.mcu_tab, text="Leak Detection")
        leak_frame.pack(fill="x", padx=10, pady=5)
        
        # Leak sensor reading
        self.leak_label = ttk.Label(leak_frame, textvariable=self.leak_var, font=("Arial", 12))
        self.leak_label.pack(anchor="w", padx=5, pady=2)
        
        # Motor current frame
        motor_frame = ttk.LabelFrame(self.mcu_tab, text="Motor Currents")
        motor_frame.pack(fill="x", padx=10, pady=5)
        
        # Motor current readings
        ttk.Label(motor_frame, textvariable=self.motor1_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(motor_frame, textvariable=self.motor2_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(motor_frame, textvariable=self.motor3_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
        ttk.Label(motor_frame, textvariable=self.motor4_var, font=("Arial", 12)).pack(anchor="w", padx=5, pady=2)
    
    def _create_control_panel(self):
        """Create quick control buttons"""
        control_frame = ttk.LabelFrame(self.status_tab, text="Quick Controls")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(pady=5)
        
        ttk.Button(button_frame, text="Forward", command=lambda: self.send_quick_command("move 0.5 0.0 0.0 0.0 0.0")).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(button_frame, text="Stop", command=lambda: self.send_quick_command("move 0.0 0.0 0.0 0.0 0.0")).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(button_frame, text="Left", command=lambda: self.send_quick_command("move 0.0 -0.5 0.0 0.0 0.0")).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(button_frame, text="Right", command=lambda: self.send_quick_command("move 0.0 0.5 0.0 0.0 0.0")).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(button_frame, text="Reverse", command=lambda: self.send_quick_command("move -0.5 0.0 0.0 0.0 0.0")).grid(row=2, column=1, padx=5, pady=5)
        
        # Depth control
        depth_frame = ttk.Frame(control_frame)
        depth_frame.pack(pady=5)
        
        ttk.Label(depth_frame, text="Depth:").grid(row=0, column=0, padx=5)
        ttk.Button(depth_frame, text="Down", command=lambda: self.send_quick_command("move 0.0 0.0 1.0 0.0 0.0")).grid(row=0, column=1, padx=5)
        ttk.Button(depth_frame, text="Up", command=lambda: self.send_quick_command("move 0.0 0.0 -1.0 0.0 0.0")).grid(row=0, column=2, padx=5)
        
        # Lights control
        lights_frame = ttk.Frame(control_frame)
        lights_frame.pack(pady=5)
        
        ttk.Label(lights_frame, text="Lights:").grid(row=0, column=0, padx=5)
        ttk.Button(lights_frame, text="On", command=lambda: self.send_quick_command("move 0.0 0.0 0.0 1.0 0.0")).grid(row=0, column=1, padx=5)
        ttk.Button(lights_frame, text="Off", command=lambda: self.send_quick_command("move 0.0 0.0 0.0 0.0 0.0")).grid(row=0, column=2, padx=5)
    
    def send_quick_command(self, cmd):
        """Send a quick command to the CommandShell"""
        global client_connection
        
        if client_connection is None:
            return
            
        parts = cmd.split()
        
        if parts[0].lower() == "move" and len(parts) >= 6:
            # Pack and send movement command
            timestamp = datetime.now().timestamp()
            timestamp_bytes = struct.pack('!d', timestamp)
            cmd_type = 1  # movement command
            cmd_data = struct.pack('!fffff', 
                                 float(parts[1]),  # throttle
                                 float(parts[2]),  # steering
                                 float(parts[3]),  # depth
                                 float(parts[4]),  # lights
                                 float(parts[5]))  # camera_tilt
            
            # Full command packet
            cmd_packet = timestamp_bytes + bytes([cmd_type]) + cmd_data
            
            with conn_lock:
                if client_connection:
                    # Send command size first
                    cmd_size = len(cmd_packet)
                    client_connection.sendall(struct.pack('!I', cmd_size))
                    
                    # Then send the actual command
                    client_connection.sendall(cmd_packet)
    
    def update_status(self):
        """Update the status display with latest data"""
        global latest_status, latest_mcu_data
        
        if latest_status is not None:
            # Update basic status
            self.depth_var.set(f"Depth: {latest_status['depth']:.2f} m")
            self.temp_var.set(f"Temperature: {latest_status['temperature']:.1f} °C")
            self.battery_var.set(f"Battery: {latest_status['battery']:.1f}%")
            
            # Update orientation
            self.pitch_var.set(f"Pitch: {latest_status['imu_pitch']:.1f}°")
            self.roll_var.set(f"Roll: {latest_status['imu_roll']:.1f}°")
            self.yaw_var.set(f"Yaw: {latest_status['imu_yaw']:.1f}°")
            
            # Update warnings
            if latest_status['water_detected']:
                self.water_var.set("WATER DETECTED!")
                self.water_label.configure(foreground="red")
            else:
                self.water_var.set("Water: Not Detected")
                self.water_label.configure(foreground="")
            
            # Update artificial horizon
            self._draw_artificial_horizon(latest_status['imu_pitch'], latest_status['imu_roll'])
        
        if latest_mcu_data is not None:
            # Update MCU data
            self.pressure_var.set(f"Pressure: {latest_mcu_data['pressure']:.2f} hPa")
            self.internal_temp_var.set(f"Internal Temp: {latest_mcu_data['internal_temp']:.1f} °C")
            self.humidity_var.set(f"Humidity: {latest_mcu_data['humidity']:.1f}%")
            
            # Update leak sensor with color warning
            leak_value = latest_mcu_data['leak_sensor']
            self.leak_var.set(f"Leak Sensor: {leak_value:.3f} V")
            if leak_value > 0.01:
                self.leak_label.configure(foreground="red")
            else:
                self.leak_label.configure(foreground="")
            
            # Update motor currents
            self.motor1_var.set(f"Motor 1 Current: {latest_mcu_data['motor_current_1']:.2f} A")
            self.motor2_var.set(f"Motor 2 Current: {latest_mcu_data['motor_current_2']:.2f} A")
            self.motor3_var.set(f"Motor 3 Current: {latest_mcu_data['motor_current_3']:.2f} A")
            self.motor4_var.set(f"Motor 4 Current: {latest_mcu_data['motor_current_4']:.2f} A")
        
        # Schedule next update
        self.root.after(100, self.update_status)
    
    def _draw_artificial_horizon(self, pitch, roll):
        """Draw an artificial horizon on the canvas based on pitch and roll values"""
        # Clear canvas
        self.horizon_canvas.delete("all")
        
        # Canvas dimensions
        width = 200
        height = 200
        center_x = width / 2
        center_y = height / 2
        radius = 80
        
        # Draw outer circle
        self.horizon_canvas.create_oval(center_x - radius, center_y - radius, 
                                      center_x + radius, center_y + radius, 
                                      outline="white", width=2)
        
        # Convert degrees to radians
        roll_rad = np.radians(roll)
        pitch_offset = pitch * 2  # Scale pitch effect
        
        # Calculate horizon line points
        x1 = center_x - radius * np.cos(roll_rad)
        y1 = center_y - radius * np.sin(roll_rad) - pitch_offset
        x2 = center_x + radius * np.cos(roll_rad)
        y2 = center_y + radius * np.sin(roll_rad) - pitch_offset
        
        # Draw horizon line
        self.horizon_canvas.create_line(x1, y1, x2, y2, fill="#00ff00", width=2)
        
        # Draw crosshair
        self.horizon_canvas.create_line(center_x - 10, center_y, center_x + 10, center_y, fill="white")
        self.horizon_canvas.create_line(center_x, center_y - 10, center_x, center_y + 10, fill="white")

class VideoFeed:
    """Separate window for displaying video feed"""
    def __init__(self, root):
        self.root = root
        self.root.title("Submersible Camera Feed")
        self.root.geometry("640x480")
        
        # Create video frame
        self.video_frame = ttk.Frame(root)
        self.video_frame.pack(fill=tk.BOTH, expand=True)
        
        # Video canvas
        self.canvas = tk.Canvas(self.video_frame, bg="black")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Text for when no video is available
        self.canvas.create_text(320, 240, text="Waiting for video feed...", fill="white", font=("Arial", 14))
        
        # Set update interval
        self.root.after(100, self.update_frame)  # Start with a slightly longer delay for the first frame
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Debug counter
        self.frame_count = 0
        self.last_log_time = time.time()
    
    def update_frame(self):
        """Update the video frame with latest received frame"""
        global latest_frame
        
        # Debug logging
        current_time = time.time()
        if current_time - self.last_log_time > 5:  # Log every 5 seconds
            print(f"Debug: Checking for frame. latest_frame is {'available' if latest_frame is not None else 'None'}")
            self.last_log_time = current_time
        
        if latest_frame is not None:
            # Get canvas size
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            
            # Ensure canvas has valid dimensions
            if canvas_width <= 1 or canvas_height <= 1:
                canvas_width = 640
                canvas_height = 480
            
            # Resize frame to fit canvas
            frame_height, frame_width = latest_frame.shape[:2]
            scale = min(canvas_width / frame_width, canvas_height / frame_height)
            new_width = int(frame_width * scale)
            new_height = int(frame_height * scale)
            
            if new_width > 0 and new_height > 0:
                try:
                    resized_frame = cv2.resize(latest_frame, (new_width, new_height))
                    # Convert from BGR to RGB for PIL
                    rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
                    
                    # Convert to PhotoImage
                    image = Image.fromarray(rgb_frame)
                    photo_image = ImageTk.PhotoImage(image=image)
                    
                    # Update canvas
                    self.canvas.delete("all")
                    self.canvas.create_image(canvas_width//2, canvas_height//2, image=photo_image, anchor=tk.CENTER)
                    self.canvas.image = photo_image  # Keep a reference to prevent garbage collection
                    
                    # Log frame count
                    self.frame_count += 1
                    if self.frame_count % 30 == 0:  # Log every 30 frames
                        print(f"Displayed frame #{self.frame_count}")
                except Exception as e:
                    print(f"Error displaying frame: {e}")
        
        # Schedule next update
        self.root.after(33, self.update_frame)
    
    def on_close(self):
        """Handle window close event"""
        # Don't close the window, just minimize it
        self.root.iconify()

class CommandShell(cmd.Cmd):
    """Interactive command processor for submersible control"""
    intro = 'Welcome to the submersible control shell. Type help or ? to list commands.\n'
    prompt = 'submersible> '
    
    def __init__(self):
        super().__init__()
        self.last_command = None
    
    def do_exit(self, arg):
        """Exit the program"""
        print("Exiting...")
        global stop_event
        stop_event.set()
        return True
    
    def do_move(self, arg):
        """Control submersible movement: move <throttle> <steering> <depth> <lights> <camera_tilt>"""
        args = arg.split()
        if len(args) != 5:
            print("Usage: move <throttle> <steering> <depth> <lights> <camera_tilt>")
            print("All values should be between -1.0 and 1.0")
            return
            
        try:
            throttle = float(args[0])
            steering = float(args[1])
            depth = float(args[2])
            lights = float(args[3])
            camera_tilt = float(args[4])
            
            # Pack and send movement command
            self._send_movement_command(throttle, steering, depth, lights, camera_tilt)
            
            print(f"Moving with throttle={throttle}, steering={steering}, depth={depth}, lights={lights}, tilt={camera_tilt}")
            
        except ValueError:
            print("All parameters must be numeric values")
    
    def do_config(self, arg):
        """Configure submersible parameters: config <parameter>=<value>"""
        if not arg:
            print("Usage: config <parameter>=<value>")
            return
            
        # Pack and send configuration command
        self._send_config_command(arg)
        
        print(f"Sent configuration: {arg}")
    
    def do_status(self, arg):
        """Display current status information"""
        global latest_status, latest_mcu_data
        
        if latest_status is None:
            print("No status data available yet")
            return
            
        print("\n===== Submersible Status =====")
        print(f"Depth: {latest_status['depth']:.2f} m")
        print(f"Temperature: {latest_status['temperature']:.1f} °C")
        print(f"Battery: {latest_status['battery']:.1f}%")
        print(f"Water Detection: {'ALERT!' if latest_status['water_detected'] else 'OK'}")
        print(f"Orientation: Pitch={latest_status['imu_pitch']:.1f}°, Roll={latest_status['imu_roll']:.1f}°, Yaw={latest_status['imu_yaw']:.1f}°")
        
        if latest_mcu_data is not None:
            print("\n===== MCU Data =====")
            print(f"Pressure: {latest_mcu_data['pressure']:.2f} hPa")
            print(f"Internal Temp: {latest_mcu_data['internal_temp']:.1f} °C")
            print(f"Humidity: {latest_mcu_data['humidity']:.1f}%")
            print(f"Leak Sensor: {latest_mcu_data['leak_sensor']:.3f} V")
            print(f"Motor Currents: {latest_mcu_data['motor_current_1']:.2f}A, {latest_mcu_data['motor_current_2']:.2f}A, {latest_mcu_data['motor_current_3']:.2f}A, {latest_mcu_data['motor_current_4']:.2f}A")
    
    def _send_movement_command(self, throttle, steering, depth, lights, camera_tilt):
        """Send a movement command to the submersible"""
        global client_connection
        
        if client_connection is None:
            print("Not connected to submersible")
            return
        
        # Pack and send movement command
        timestamp = datetime.now().timestamp()
        timestamp_bytes = struct.pack('!d', timestamp)
        cmd_type = 1  # movement command
        cmd_data = struct.pack('!fffff', 
                             float(throttle),
                             float(steering),
                             float(depth),
                             float(lights),
                             float(camera_tilt))
        
        # Full command packet
        cmd_packet = timestamp_bytes + bytes([cmd_type]) + cmd_data
        
        with conn_lock:
            if client_connection:
                # Send command size first
                cmd_size = len(cmd_packet)
                client_connection.sendall(struct.pack('!I', cmd_size))
                
                # Then send the actual command
                client_connection.sendall(cmd_packet)
    
    def _send_config_command(self, config_str):
        """Send a configuration command to the submersible"""
        global client_connection
        
        if client_connection is None:
            print("Not connected to submersible")
            return
        
        # Pack and send configuration command
        timestamp = datetime.now().timestamp()
        timestamp_bytes = struct.pack('!d', timestamp)
        cmd_type = 2  # configuration command
        cmd_data = config_str.encode('utf-8')
        
        # Full command packet
        cmd_packet = timestamp_bytes + bytes([cmd_type]) + cmd_data
        
        with conn_lock:
            if client_connection:
                # Send command size first
                cmd_size = len(cmd_packet)
                client_connection.sendall(struct.pack('!I', cmd_size))
                
                # Then send the actual command
                client_connection.sendall(cmd_packet)
    
    # Command aliases for convenience
    do_quit = do_exit
    do_m = do_move
    do_c = do_config
    do_s = do_status

def data_receiver(connection):
    """Receive data from the submersible"""
    global latest_frame, latest_status, latest_mcu_data, frame_count

    while not stop_event.is_set():
        try:
            # First receive the packet size
            packet_size_bytes = connection.recv(4)
            if not packet_size_bytes or len(packet_size_bytes) < 4:
                print("Connection closed by client")
                break

            packet_size = struct.unpack('!I', packet_size_bytes)[0]

            # Then receive the actual packet
            packet_bytes = b''
            bytes_received = 0

            while bytes_received < packet_size:
                chunk = connection.recv(min(packet_size - bytes_received, 8192))
                if not chunk:
                    raise Exception("Connection broken")
                packet_bytes += chunk
                bytes_received += len(chunk)

            # Unpack packet
            # Format: [timestamp(8)][status_data(28)][mcu_data(32)][frame_size(4)][frame_data(variable)]

            # Extract timestamp
            timestamp = struct.unpack('!d', packet_bytes[0:8])[0]

            # Extract status data
            status_offset = 8
            status_bytes = packet_bytes[status_offset:status_offset + 28]
            status_values = struct.unpack('!fffffff', status_bytes)

            latest_status = {
                'depth': status_values[0],
                'temperature': status_values[1],
                'battery': status_values[2],
                'water_detected': status_values[3] > 0.5,
                'imu_pitch': status_values[4],
                'imu_roll': status_values[5],
                'imu_yaw': status_values[6]
            }

            # Extract MCU data (always assumed present)
            mcu_offset = status_offset + 28  # = 36
            mcu_bytes = packet_bytes[mcu_offset:mcu_offset + 32]
            mcu_values = struct.unpack('!ffffffff', mcu_bytes)

            latest_mcu_data = {
                'pressure': mcu_values[0],
                'internal_temp': mcu_values[1],
                'humidity': mcu_values[2],
                'leak_sensor': mcu_values[3],
                'motor_current_1': mcu_values[4],
                'motor_current_2': mcu_values[5],
                'motor_current_3': mcu_values[6],
                'motor_current_4': mcu_values[7]
            }

            # Extract frame size and frame data
            frame_offset = mcu_offset + 32  # = 68
            frame_size = struct.unpack('!I', packet_bytes[frame_offset:frame_offset + 4])[0]
            frame_data_offset = frame_offset + 4

            print(f"packet size: {len(packet_bytes)}")
            print(f"frame_offset: {frame_offset}")
            print(f"frame_size: {frame_size}")
            print(f"frame_data_offset + frame_size = {frame_data_offset + frame_size}")
            print("frame size bytes:", packet_bytes[frame_offset:frame_offset + 4])

            if len(packet_bytes) >= frame_data_offset + frame_size and frame_size > 0:
                print("frame exists")
                frame_data = packet_bytes[frame_data_offset:frame_data_offset + frame_size]

                # Decode frame
                frame_array = np.frombuffer(frame_data, dtype=np.uint8)
                decoded_frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                cv2.imshow("Decoded image", decoded_frame)

                if decoded_frame is not None:
                    latest_frame = decoded_frame
                    frame_count += 1

                    # Log every 30 frames
                    if frame_count % 30 == 0:
                        now = datetime.now().timestamp()
                        latency = (now - timestamp) * 1000  # in ms
                        print(f"Received frame {frame_count}, latency: {latency:.2f} ms")

        except Exception as e:
            print(f"Error receiving data: {e}")
            break

    # Clean up
    with conn_lock:
        connection.close()
        global client_connection
        client_connection = None
        print("Connection closed and cleaned up.")


def connect_to_submersible(host, port):
    """Connect to the submersible device"""
    global client_connection
    
    try:
        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to submersible at {host}:{port}...")
        sock.connect((host, port))
        print("Connected successfully!")
        
        with conn_lock:
            client_connection = sock
        
        # Start receiver thread
        receiver_thread = threading.Thread(target=data_receiver, args=(sock,))
        receiver_thread.daemon = True
        receiver_thread.start()
        
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        return False

def main():
    global stop_event
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Surface vessel control station')
    parser.add_argument('--host', default='192.168.1.100', help='Listen address 192.168.1.100')
    parser.add_argument('--port', type=int, default=8000, help='Listen port')
    parser.add_argument('--connect', action='store_true', help='Connect to submersible instead of listening')
    parser.add_argument('--submersible-ip', default='192.168.1.101', help='Submersible IP when using --connect')
    args = parser.parse_args()
    
    # Create root window for Tkinter
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    
    # Create video feed window
    video_window = tk.Toplevel(root)
    video_feed = VideoFeed(video_window)
    
    # Create status display window
    status_window = tk.Toplevel(root)
    status_display = StatusDisplay(status_window)
    
    # Start a server or connect to the submersible
    if args.connect:
        # Original client mode
        connected = connect_to_submersible(args.submersible_ip, args.port)
        if not connected:
            print("Could not connect to submersible. Make sure it's running and try again.")
    else:
        # New server mode
        server_thread = threading.Thread(target=start_server, args=(args.host, args.port))
        server_thread.daemon = True
        server_thread.start()
        print(f"Server started, listening on {args.host}:{args.port}")
        print("Waiting for submersible to connect...")
    
    # Create and start the command shell
    shell_thread = threading.Thread(target=lambda: CommandShell().cmdloop())
    shell_thread.daemon = True
    shell_thread.start()
    
    try:
        # Start Tkinter main loop
        root.mainloop()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        stop_event.set()
        if client_connection:
            with conn_lock:
                client_connection.close()
        print("Program terminated.")

def start_server(host, port):
    """Start a server to accept connections from the submersible"""
    global client_connection, stop_event
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((host, port))
        server_socket.listen(1)
        
        while not stop_event.is_set():
            print("Waiting for submersible to connect...")
            try:
                conn, addr = server_socket.accept()
                print(f"Submersible connected from {addr}")
                
                with conn_lock:
                    client_connection = conn
                
                # Start receiver thread for this connection
                receiver_thread = threading.Thread(target=data_receiver, args=(conn,))
                receiver_thread.daemon = True
                receiver_thread.start()
                
                # Wait until this connection is closed before accepting another
                while client_connection and not stop_event.is_set():
                    time.sleep(1)
                    
            except Exception as e:
                if not stop_event.is_set():
                    print(f"Error accepting connection: {e}")
                    time.sleep(5)  # Wait before trying again
    
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()