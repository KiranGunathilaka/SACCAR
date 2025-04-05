import socket
import cv2
import time
import subprocess
import threading
import os
import struct
import numpy as np
import random
from datetime import datetime

# Global variables
current_frame = None
stop_event = threading.Event()
control_data = {
    'throttle': 0.0,
    'steering': 0.0,
    'depth': 0.0,
    'lights': 0,
    'camera_tilt': 0.0
}
status_data = {
    'depth': 0.0,
    'temperature': 22.5,
    'battery': 95.0,
    'water_detected': False,
    'imu_pitch': 0.0,
    'imu_roll': 0.0,
    'imu_yaw': 0.0
}
mcu_data = {
    'pressure': 1013.25,     # hPa
    'internal_temp': 35.0,   # Celsius
    'humidity': 45.0,        # Percent
    'leak_sensor': 0.0,      # Voltage (higher = possible leak)
    'motor_current_1': 0.0,  # Amps
    'motor_current_2': 0.0,  # Amps
    'motor_current_3': 0.0,  # Amps
    'motor_current_4': 0.0   # Amps
}

def generate_dummy_mcu_data():
    """Generate varying dummy data simulating MCU sensor readings"""
    global mcu_data
    while not stop_event.is_set():
        # Add random variations to simulate real sensor readings
        mcu_data['pressure'] = 1013.25 + random.uniform(-2.0, 2.0)
        mcu_data['internal_temp'] = 35.0 + random.uniform(-0.5, 0.5)
        mcu_data['humidity'] = 45.0 + random.uniform(-2.0, 2.0)
        mcu_data['leak_sensor'] = max(0.0, random.uniform(-0.01, 0.02))
        
        # Simulate motor currents based on throttle command
        base_current = abs(control_data['throttle']) * 2.0
        mcu_data['motor_current_1'] = base_current + random.uniform(-0.2, 0.2)
        mcu_data['motor_current_2'] = base_current + random.uniform(-0.2, 0.2)
        mcu_data['motor_current_3'] = abs(control_data['steering']) * 1.5 + random.uniform(-0.1, 0.1)
        mcu_data['motor_current_4'] = abs(control_data['steering']) * 1.5 + random.uniform(-0.1, 0.1)
        
        time.sleep(0.1)  # Update 10 times per second

def capture_frames():
    """Capture frames using libcamera-vid and pipe to OpenCV"""
    cmd = "libcamera-vid -t 0 --width 640 --height 480 --codec yuv420 --output -"
    process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
    frame_size = 640 * 480 * 3 // 2  # YUV420 format
    
    try:
        while not stop_event.is_set():
            # Read raw frame data
            raw_data = process.stdout.read(frame_size)
            if len(raw_data) < frame_size:
                break
                
            # Convert YUV420 to BGR
            yuv = np.frombuffer(raw_data, dtype=np.uint8).reshape((480 * 3 // 2, 640))
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            
            # Put frame in the global variable
            global current_frame
            current_frame = frame
            
            # No local display needed for submersible
    finally:
        process.terminate()

def data_sender(client_socket):
    """Send camera frames, status data, and MCU data to server"""
    frame_count = 0
    
    while not stop_event.is_set():
        if current_frame is not None:
            # Compress frame as JPEG for network transmission
            _, frame_encoded = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            frame_data = frame_encoded.tobytes()
            
            # Update status data (would be from actual sensors in real implementation)
            global status_data
            status_data['depth'] = control_data['depth']  # Simulate depth following command
            status_data['imu_pitch'] += 0.01  # Simulate changing sensor values
            if status_data['imu_pitch'] > 10.0:
                status_data['imu_pitch'] = -10.0
            
            # Simulate battery drain
            status_data['battery'] -= 0.001
            if status_data['battery'] < 10.0:
                status_data['battery'] = 10.0
            
            # Serialize status data
            status_bytes = struct.pack('!fffffff', 
                                     status_data['depth'],
                                     status_data['temperature'],
                                     status_data['battery'],
                                     1.0 if status_data['water_detected'] else 0.0,
                                     status_data['imu_pitch'],
                                     status_data['imu_roll'],
                                     status_data['imu_yaw'])
            
            # Serialize MCU data
            mcu_bytes = struct.pack('!ffffffff',
                                   mcu_data['pressure'],
                                   mcu_data['internal_temp'],
                                   mcu_data['humidity'],
                                   mcu_data['leak_sensor'],
                                   mcu_data['motor_current_1'],
                                   mcu_data['motor_current_2'],
                                   mcu_data['motor_current_3'],
                                   mcu_data['motor_current_4'])
            
            # Get current timestamp
            timestamp = datetime.now().timestamp()
            timestamp_bytes = struct.pack('!d', timestamp)
            
            # Create custom packet:
            # Format: [timestamp(8)][status_data(28)][mcu_data(32)][frame_size(4)][frame_data(variable)]
            frame_size = len(frame_data)
            frame_size_bytes = struct.pack('!I', frame_size)
            
            # Combine all parts into a packet
            packet = timestamp_bytes + status_bytes + mcu_bytes + frame_size_bytes + frame_data
            
            try:
                # Send the packet size first
                packet_size = len(packet)
                client_socket.sendall(struct.pack('!I', packet_size))
                
                # Then send the actual packet
                client_socket.sendall(packet)
                
                frame_count += 1
                if frame_count % 100 == 0:  # Print status less frequently to reduce console output
                    print(f"Sent frame {frame_count}, size: {frame_size} bytes")
                    print(f"MCU data: pressure={mcu_data['pressure']:.2f}hPa, temp={mcu_data['internal_temp']:.1f}°C")
                
            except Exception as e:
                print(f"Error sending frame: {e}")
                break
                
        time.sleep(0.033)  # ~30 FPS

def control_receiver(client_socket):
    """Listen for incoming control commands from the server"""
    global control_data
    
    while not stop_event.is_set():
        try:
            # First receive the command size
            cmd_size_bytes = client_socket.recv(4)
            if not cmd_size_bytes or len(cmd_size_bytes) < 4:
                print("Connection closed by server")
                break
                
            cmd_size = struct.unpack('!I', cmd_size_bytes)[0]
            
            # Then receive the actual command
            cmd_bytes = b''
            bytes_received = 0
            
            while bytes_received < cmd_size:
                chunk = client_socket.recv(min(cmd_size - bytes_received, 8192))
                if not chunk:
                    raise Exception("Connection broken")
                cmd_bytes += chunk
                bytes_received += len(chunk)
            
            # Unpack command
            # Format: [timestamp(8 bytes)][command_type(1 byte)][command_data(variable)]
            timestamp = struct.unpack('!d', cmd_bytes[0:8])[0]
            cmd_type = cmd_bytes[8]
            cmd_data = cmd_bytes[9:]
            
            # Process command based on type
            if cmd_type == 1:  # Movement command
                new_control = struct.unpack('!fffff', cmd_data)
                control_data['throttle'] = new_control[0]
                control_data['steering'] = new_control[1]
                control_data['depth'] = new_control[2]
                control_data['lights'] = new_control[3]
                control_data['camera_tilt'] = new_control[4]
                
                print(f"COMMAND: throttle={control_data['throttle']:.2f}, steering={control_data['steering']:.2f}, depth={control_data['depth']:.2f}")
                
            elif cmd_type == 2:  # Configuration command
                config_str = cmd_data.decode('utf-8')
                print(f"CONFIG: {config_str}")
                
                # Process configuration command
                try:
                    parts = config_str.split('=')
                    if len(parts) == 2:
                        param, value = parts
                        # Handle different configuration parameters
                        if param == "video_quality":
                            quality = int(value)
                            print(f"Setting video quality to {quality}%")
                            # You would apply this setting to your video encoding
                        # Add more configuration parameters as needed
                except Exception as e:
                    print(f"Error processing configuration: {e}")
                
            else:
                print(f"Unknown command type: {cmd_type}")
                
        except Exception as e:
            print(f"Error receiving command: {e}")
            if "Broken pipe" in str(e) or "Connection reset" in str(e):
                break

def start_client():
    global stop_event
    
    # Start frame capture in a separate thread
    capture_thread = threading.Thread(target=capture_frames)
    capture_thread.daemon = True
    capture_thread.start()
    
    # Start dummy MCU data generation thread
    mcu_thread = threading.Thread(target=generate_dummy_mcu_data)
    mcu_thread.daemon = True
    mcu_thread.start()
    
    # Wait for first frame
    print("Waiting for camera to initialize...")
    while current_frame is None and not stop_event.is_set():
        time.sleep(0.1)
    
    print("Camera initialized, connecting to server...")
    
    # Connect to server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect(('192.168.1.100', 8000))  # Server IP address
        print("Connected to surface vessel")
        
        # Start threads for sending frames/status and receiving commands
        sender_thread = threading.Thread(target=data_sender, args=(client_socket,))
        receiver_thread = threading.Thread(target=control_receiver, args=(client_socket,))
        
        sender_thread.daemon = True
        receiver_thread.daemon = True
        
        sender_thread.start()
        receiver_thread.start()
        
        # Main loop to keep program running
        print("Submersible running. Press Ctrl+C to exit.")
        try:
            while not stop_event.is_set():
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
            stop_event.set()
            
    except Exception as e:
        print(f"Connection error: {e}")
        stop_event.set()
    finally:
        # Clean up resources
        stop_event.set()
        client_socket.close()
        print("Submersible stopped")

if __name__ == "__main__":
    start_client()