import socket
import cv2
import threading
import struct
import numpy as np
import time
from datetime import datetime

# Global variables
latest_frame = None
latest_status = None
frame_count = 0
stop_event = threading.Event()

def handle_client(conn, addr):
    """Handle client connection - receive frames/status and send control commands"""
    global latest_frame, latest_status, frame_count
    
    print(f"Connected by {addr}")
    
    # Start a thread to display frames
    display_thread = threading.Thread(target=display_frames)
    display_thread.daemon = True
    display_thread.start()
    
    # Start a thread to handle user input for commands
    cmd_thread = threading.Thread(target=control_input, args=(conn,))
    cmd_thread.daemon = True
    cmd_thread.start()
    
    try:
        while not stop_event.is_set():
            try:
                # First receive the packet size
                size_data = conn.recv(4)
                if not size_data or len(size_data) < 4:
                    print("Connection closed by client.")
                    break
                    
                packet_size = struct.unpack('!I', size_data)[0]
                
                # Receive the full packet
                packet = b''
                bytes_received = 0
                
                while bytes_received < packet_size:
                    chunk = conn.recv(min(packet_size - bytes_received, 8192))
                    if not chunk:
                        raise Exception("Connection broken")
                    packet += chunk
                    bytes_received += len(chunk)
                
                # Parse the packet
                # Format: [timestamp(8 bytes)][status_data(28 bytes)][frame_size(4 bytes)][frame_data(variable)]
                timestamp = struct.unpack('!d', packet[0:8])[0]
                
                # Unpack status data
                status_data = struct.unpack('!fffffff', packet[8:36])
                status_dict = {
                    'depth': status_data[0],
                    'temperature': status_data[1],
                    'battery': status_data[2],
                    'water_detected': status_data[3] > 0.5,
                    'imu_pitch': status_data[4],
                    'imu_roll': status_data[5],
                    'imu_yaw': status_data[6]
                }
                latest_status = status_dict
                
                # Unpack frame size and data
                frame_size = struct.unpack('!I', packet[36:40])[0]
                frame_data = packet[40:40+frame_size]
                
                # Decode the frame
                frame_array = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    latest_frame = frame
                    frame_count += 1
                    
                    # Calculate latency
                    now = datetime.now().timestamp()
                    latency = (now - timestamp) * 1000  # in ms
                    
                    # if frame_count % 30 == 0:  # Print status every 30 frames
                    #     print(f"Received frame {frame_count}, latency: {latency:.2f} ms")
                    #     print(f"Status: depth={status_dict['depth']:.1f}m, temp={status_dict['temperature']:.1f}°C, "
                    #           f"battery={status_dict['battery']:.1f}%, water={status_dict['water_detected']}")
                
            except Exception as e:
                print(f"Error processing packet: {e}")
                break
                
    finally:
        conn.close()
        print(f"Connection from {addr} closed")

def display_frames():
    """Display received frames in a window with telemetry overlay"""
    cv2.namedWindow("Submersible Camera Feed", cv2.WINDOW_NORMAL)
    
    while not stop_event.is_set():
        if latest_frame is not None and latest_status is not None:
            # Display frame with telemetry overlay
            frame_copy = latest_frame.copy()
            
            # Add telemetry overlay
            cv2.putText(frame_copy, f"Depth: {latest_status['depth']:.1f}m", (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame_copy, f"Temp: {latest_status['temperature']:.1f}°C", (10, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame_copy, f"Battery: {latest_status['battery']:.1f}%", (10, 90), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add attitude indicator
            pitch = latest_status['imu_pitch']
            roll = latest_status['imu_roll']
            
            # Draw horizon line based on pitch and roll
            center_x, center_y = 570, 60
            radius = 40
            horizon_length = 70
            
            # Calculate horizon line endpoints based on roll angle
            x1 = int(center_x - horizon_length/2 * np.cos(np.radians(roll)))
            y1 = int(center_y - horizon_length/2 * np.sin(np.radians(roll)) - pitch)
            x2 = int(center_x + horizon_length/2 * np.cos(np.radians(roll)))
            y2 = int(center_y + horizon_length/2 * np.sin(np.radians(roll)) - pitch)
            
            # Draw attitude indicator
            cv2.circle(frame_copy, (center_x, center_y), radius, (255, 255, 255), 1)
            cv2.line(frame_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.line(frame_copy, (center_x - 10, center_y), (center_x + 10, center_y), (0, 255, 0), 1)
            cv2.line(frame_copy, (center_x, center_y - 10), (center_x, center_y + 10), (0, 255, 0), 1)
            
            # Warning indicators
            if latest_status['water_detected']:
                cv2.putText(frame_copy, "WATER DETECTED!", (210, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
            if latest_status['battery'] < 20.0:
                cv2.putText(frame_copy, "LOW BATTERY!", (250, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow("Submersible Camera Feed", frame_copy)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break
            
        time.sleep(0.01)
    
    cv2.destroyAllWindows()

def control_input(conn):
    """Handle user input to send control commands to the client"""
    print("Control interface ready. Type commands or press Ctrl+C to exit.")
    print("Available commands:")
    print("  move <throttle> <steering> <depth> <lights> <camera_tilt> - Send movement command")
    print("      throttle: -1.0 to 1.0, steering: -1.0 to 1.0, depth: meters")
    print("      lights: 0 or 1, camera_tilt: -90 to 90 degrees")
    print("  config <parameter>=<value> - Send configuration command")
    print("      Example: config video_quality=80")
    
    current_controls = {
        'throttle': 0.0,
        'steering': 0.0,
        'depth': 0.0,
        'lights': 0.0,
        'camera_tilt': 0.0
    }
    
    while not stop_event.is_set():
        try:
            cmd_input = input("> ")
            parts = cmd_input.split()
            
            if not parts:
                continue
                
            if parts[0].lower() == "move":
                if len(parts) >= 6:
                    # Full movement command with all parameters
                    try:
                        current_controls['throttle'] = float(parts[1])
                        current_controls['steering'] = float(parts[2])
                        current_controls['depth'] = float(parts[3])
                        current_controls['lights'] = float(parts[4])
                        current_controls['camera_tilt'] = float(parts[5])
                    except ValueError:
                        print("Invalid movement parameters.")
                        continue
                else:
                    # Partial command - show help
                    print("Usage: move <throttle> <steering> <depth> <lights> <camera_tilt>")
                    continue
                    
                # Pack and send movement command
                timestamp = datetime.now().timestamp()
                timestamp_bytes = struct.pack('!d', timestamp)
                cmd_type = 1  # movement command
                cmd_data = struct.pack('!fffff', 
                                     current_controls['throttle'],
                                     current_controls['steering'],
                                     current_controls['depth'],
                                     current_controls['lights'],
                                     current_controls['camera_tilt'])
                
                # Full command packet
                cmd_packet = timestamp_bytes + bytes([cmd_type]) + cmd_data
                
                # Send command size first
                cmd_size = len(cmd_packet)
                conn.sendall(struct.pack('!I', cmd_size))
                
                # Then send the actual command
                conn.sendall(cmd_packet)
                
                print(f"Sent control: throttle={current_controls['throttle']:.2f}, "
                      f"steering={current_controls['steering']:.2f}, depth={current_controls['depth']:.2f}, "
                      f"lights={'ON' if current_controls['lights'] > 0.5 else 'OFF'}, "
                      f"camera_tilt={current_controls['camera_tilt']:.2f}")
                
            elif parts[0].lower() == "config":
                if len(parts) < 2:
                    print("Usage: config <parameter>=<value>")
                    continue
                    
                config_str = ' '.join(parts[1:])
                
                # Pack and send configuration command
                timestamp = datetime.now().timestamp()
                timestamp_bytes = struct.pack('!d', timestamp)
                cmd_type = 2  # config command
                cmd_data = config_str.encode('utf-8')
                
                # Full command packet
                cmd_packet = timestamp_bytes + bytes([cmd_type]) + cmd_data
                
                # Send command size first
                cmd_size = len(cmd_packet)
                conn.sendall(struct.pack('!I', cmd_size))
                
                # Then send the actual command
                conn.sendall(cmd_packet)
                
                print(f"Sent config: {config_str}")
                
            # Keyboard shortcut commands for quick control
            elif parts[0].lower() == "forward":
                current_controls['throttle'] = 0.5
                # Send movement command with updated controls
                # (similar code as in the "move" command case)
                
            elif parts[0].lower() == "stop":
                current_controls['throttle'] = 0.0
                current_controls['steering'] = 0.0
                # Send movement command with updated controls
                
            elif parts[0].lower() == "help":
                print("Available commands:")
                print("  move <throttle> <steering> <depth> <lights> <camera_tilt>")
                print("  config <parameter>=<value> (Example: config video_quality=80)")
                print("  forward - Quick command to move forward (throttle=0.5)")
                print("  stop - Quick command to stop all movement")
                
            else:
                print("Unknown command. Type 'help' for available commands.")
                
        except Exception as e:
            print(f"Error sending command: {e}")
            if "Broken pipe" in str(e) or "Connection reset" in str(e):
                break

def start_server():
    global stop_event
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('192.168.1.100', 8000))
    server_socket.listen(1)
    
    print("Surface vessel server started. Waiting for submersible to connect...")
    
    try:
        conn, addr = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(conn, addr))
        client_thread.daemon = True
        client_thread.start()
        
        # Keep the main thread running
        try:
            while not stop_event.is_set():
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
            stop_event.set()
            
    finally:
        server_socket.close()
        print("Server stopped")

if __name__ == "__main__":
    start_server()