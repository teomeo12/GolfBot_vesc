#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import tkinter as tk
from tkinter import ttk
import numpy as np
from PIL import Image as PILImage, ImageTk
import threading
import queue

class GolfBotGUI(Node):
    def __init__(self):
        super().__init__('golfbot_gui_node')
        
        # ROS setup
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        
        # Data storage
        self.latest_image = None
        self.gps_data = None
        self.odom_data = None
        self.autonomous_mode = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
            
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        self.autonomous_mode_sub = self.create_subscription(
            Bool,
            '/is_autonomous_mode',
            self.autonomous_mode_callback,
            10)
        
        # Publishers
        self.autonomous_mode_pub = self.create_publisher(Bool, '/is_autonomous_mode', 10)
        
        # GUI setup
        self.root = tk.Tk()
        self.root.title("GolfBot Control Panel")
        self.root.geometry("1000x700")
        
        self.setup_gui()
        
        # Start GUI update timer
        self.gui_timer = self.create_timer(0.1, self.update_gui)
        
        self.get_logger().info('GolfBot GUI Node initialized')
    
    def setup_gui(self):
        """Setup the main GUI layout"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
        # Left side - Camera view
        camera_frame = ttk.LabelFrame(main_frame, text="Camera View", padding="10")
        camera_frame.grid(row=0, column=0, padx=(0, 10), sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.camera_label = ttk.Label(camera_frame, text="No camera feed")
        self.camera_label.pack(expand=True, fill='both')
        
        # Right side - Data and controls
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Autonomous mode control
        auto_frame = ttk.LabelFrame(control_frame, text="Autonomous Control", padding="10")
        auto_frame.pack(fill='x', pady=(0, 10))
        
        self.auto_status_label = ttk.Label(auto_frame, text="Status: Manual Mode", font=("Arial", 12, "bold"))
        self.auto_status_label.pack(pady=(0, 10))
        
        self.auto_button = ttk.Button(auto_frame, text="Enable Auto Mode", command=self.toggle_autonomous_mode)
        self.auto_button.pack()
        
        # GPS Data
        gps_frame = ttk.LabelFrame(control_frame, text="GPS Status", padding="10")
        gps_frame.pack(fill='x', pady=(0, 10))
        
        self.gps_lat_label = ttk.Label(gps_frame, text="Latitude: --")
        self.gps_lat_label.pack(anchor='w')
        
        self.gps_lon_label = ttk.Label(gps_frame, text="Longitude: --")
        self.gps_lon_label.pack(anchor='w')
        
        self.gps_alt_label = ttk.Label(gps_frame, text="Altitude: --")
        self.gps_alt_label.pack(anchor='w')
        
        self.gps_status_label = ttk.Label(gps_frame, text="Fix Status: --")
        self.gps_status_label.pack(anchor='w')
        
        # Odometry Data  
        odom_frame = ttk.LabelFrame(control_frame, text="Robot Position", padding="10")
        odom_frame.pack(fill='x', pady=(0, 10))
        
        self.odom_x_label = ttk.Label(odom_frame, text="X: --")
        self.odom_x_label.pack(anchor='w')
        
        self.odom_y_label = ttk.Label(odom_frame, text="Y: --")
        self.odom_y_label.pack(anchor='w')
        
        self.odom_theta_label = ttk.Label(odom_frame, text="Heading: --")
        self.odom_theta_label.pack(anchor='w')
        
        # System Status
        status_frame = ttk.LabelFrame(control_frame, text="System Status", padding="10")
        status_frame.pack(fill='x')
        
        self.camera_status_label = ttk.Label(status_frame, text="Camera: Disconnected", foreground="red")
        self.camera_status_label.pack(anchor='w')
        
        self.gps_conn_label = ttk.Label(status_frame, text="GPS: Disconnected", foreground="red")
        self.gps_conn_label.pack(anchor='w')
        
        self.odom_conn_label = ttk.Label(status_frame, text="Odometry: Disconnected", foreground="red")
        self.odom_conn_label.pack(anchor='w')
    
    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Put image in queue (non-blocking)
            if not self.image_queue.full():
                self.image_queue.put(cv_image)
                
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {str(e)}')
    
    def gps_callback(self, msg):
        """Handle GPS data"""
        self.gps_data = msg
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        self.odom_data = msg
    
    def autonomous_mode_callback(self, msg):
        """Handle autonomous mode status updates"""
        self.autonomous_mode = msg.data
    
    def toggle_autonomous_mode(self):
        """Toggle autonomous mode on/off"""
        new_mode = not self.autonomous_mode
        
        msg = Bool()
        msg.data = new_mode
        self.autonomous_mode_pub.publish(msg)
        
        self.get_logger().info(f'Autonomous mode {"enabled" if new_mode else "disabled"}')
    
    def update_gui(self):
        """Update GUI elements with latest data"""
        try:
            # Update camera display
            if not self.image_queue.empty():
                cv_image = self.image_queue.get()
                self.display_image(cv_image)
                self.camera_status_label.config(text="Camera: Connected", foreground="green")
            
            # Update autonomous mode status
            if self.autonomous_mode:
                self.auto_status_label.config(text="Status: AUTONOMOUS MODE", foreground="red")
                self.auto_button.config(text="Disable Auto Mode")
            else:
                self.auto_status_label.config(text="Status: Manual Mode", foreground="blue")
                self.auto_button.config(text="Enable Auto Mode")
            
            # Update GPS data
            if self.gps_data is not None:
                self.gps_lat_label.config(text=f"Latitude: {self.gps_data.latitude:.6f}°")
                self.gps_lon_label.config(text=f"Longitude: {self.gps_data.longitude:.6f}°")
                self.gps_alt_label.config(text=f"Altitude: {self.gps_data.altitude:.2f}m")
                
                # GPS fix status
                status_map = {0: "No Fix", 1: "GPS Fix", 2: "DGPS Fix", 3: "PPS Fix", 
                             4: "RTK Fixed", 5: "RTK Float", 6: "Dead Reckoning", 
                             7: "Manual", 8: "Simulation"}
                status_text = status_map.get(self.gps_data.status.status, "Unknown")
                self.gps_status_label.config(text=f"Fix Status: {status_text}")
                self.gps_conn_label.config(text="GPS: Connected", foreground="green")
            else:
                self.gps_conn_label.config(text="GPS: Disconnected", foreground="red")
            
            # Update odometry data
            if self.odom_data is not None:
                x = self.odom_data.pose.pose.position.x
                y = self.odom_data.pose.pose.position.y
                
                # Calculate heading from quaternion
                q = self.odom_data.pose.pose.orientation
                heading = self.quat_to_yaw(q) * 180.0 / 3.14159  # Convert to degrees
                
                self.odom_x_label.config(text=f"X: {x:.3f}m")
                self.odom_y_label.config(text=f"Y: {y:.3f}m")
                self.odom_theta_label.config(text=f"Heading: {heading:.1f}°")
                self.odom_conn_label.config(text="Odometry: Connected", foreground="green")
            else:
                self.odom_conn_label.config(text="Odometry: Disconnected", foreground="red")
                
        except Exception as e:
            self.get_logger().error(f'GUI update error: {str(e)}')
    
    def display_image(self, cv_image):
        """Display OpenCV image in tkinter label"""
        try:
            # Resize image to fit display (maintain aspect ratio)
            height, width = cv_image.shape[:2]
            max_width = 640
            max_height = 480
            
            # Calculate scaling factor
            scale = min(max_width/width, max_height/height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            
            # Resize image
            resized = cv2.resize(cv_image, (new_width, new_height))
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image
            pil_image = PILImage.fromarray(rgb_image)
            
            # Convert to PhotoImage
            photo = ImageTk.PhotoImage(pil_image)
            
            # Update label
            self.camera_label.configure(image=photo)
            self.camera_label.image = photo  # Keep a reference
            
        except Exception as e:
            self.get_logger().error(f'Image display error: {str(e)}')
    
    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def run(self):
        """Run the GUI main loop"""
        # Start ROS spinning in a separate thread
        def spin_ros():
            rclpy.spin(self)
        
        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        # Run tkinter main loop
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    gui_node = GolfBotGUI()
    
    try:
        gui_node.run()
    except KeyboardInterrupt:
        gui_node.get_logger().info('GUI node stopped by user')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()