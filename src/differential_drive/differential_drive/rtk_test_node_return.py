import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class RTKTestNodeReturn(Node):
    def __init__(self):
        super().__init__('rtk_test_node_return')

        # Subscriptions
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.results_pub = self.create_publisher(String, 'rtk_test_results', 10)
        self.status_pub = self.create_publisher(String, 'rtk_status', 10)
        
        # State variables
        self.current_lat, self.current_lon = None, None
        self.current_yaw = None
        self.current_odom_x = None
        self.position_covariance = None
        self.start_pos_gps = None
        self.start_pos_odom = None
        self.target_heading = None
        self.state = "IDLE"
        self.prev_a_state = False
        
        # Parameters
        self.drive_speed = 550.0
        self.turn_speed = 550.0 # Using fixed turn speed
        self.heading_kp = 2.5
        self.turn_tolerance_rad = math.radians(5)
        self.distance_tolerance_m = 0.05 # 5cm for odom stop

        # Control loop timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('RTK Test Node (Return w/ Odom) Initialized')
        self.get_logger().info('Using ODOM for distance, IMU for heading, GPS for reporting.')
        self.get_logger().info('Press A Button: Start Test')

    def imu_callback(self, msg):
        q = msg.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def gps_callback(self, msg):
        self.current_lat, self.current_lon = msg.latitude, msg.longitude
        self.position_covariance = msg.position_covariance

    def odom_callback(self, msg):
        self.current_odom_x = msg.pose.pose.position.x

    def joy_callback(self, msg):
        if len(msg.buttons) < 2: return
        a_pressed = msg.buttons[0] == 1
        if a_pressed and not self.prev_a_state: self.start_rtk_test()
        self.prev_a_state = a_pressed
        b_pressed = msg.buttons[1] == 1
        if b_pressed: self.stop_rtk_test()

    def start_rtk_test(self):
        if self.state != "IDLE": return self.get_logger().warn("Test already in progress.")
        if any(v is None for v in [self.current_lat, self.current_yaw, self.current_odom_x]):
            return self.get_logger().warn("Cannot start: Missing GPS, IMU, or Odom data.")

        self.start_pos_gps = (self.current_lat, self.current_lon)
        self.start_pos_odom = self.current_odom_x
        self.target_heading = self.current_yaw
        self.state = "DRIVING_FORWARD"
        self.get_logger().info("🚀 STARTING THERE-AND-BACK TEST: State -> DRIVING_FORWARD")
        self.results_pub.publish(String(data=f"START | Lat: {self.start_pos_gps[0]:.8f}, Lon: {self.start_pos_gps[1]:.8f}"))

    def stop_rtk_test(self):
        self.state = "IDLE"
        self.send_stop_command()
        self.get_logger().info("🛑 TEST STOPPED BY USER.")

    def control_loop(self):
        if self.state == "IDLE" or any(v is None for v in [self.current_yaw, self.current_odom_x]): return

        if self.state == "DRIVING_FORWARD":
            distance_traveled = abs(self.current_odom_x - self.start_pos_odom)
            if distance_traveled < 1.0:
                self.drive_straight()
                self.publish_status(f"State: FWD (Odom) | Dist: {distance_traveled:.2f}m")
            else:
                self.state = "TURNING"
                self.target_heading = self.normalize_angle(self.current_yaw + math.pi)
                self.send_stop_command()
                self.get_logger().info(f"✅ Reached 1m (Odom). State -> TURNING to {math.degrees(self.target_heading):.1f}°")
        
        elif self.state == "TURNING":
            heading_error = self.normalize_angle(self.target_heading - self.current_yaw)
            if abs(heading_error) > self.turn_tolerance_rad:
                cmd = Twist()
                cmd.angular.z = self.turn_speed if heading_error > 0 else -self.turn_speed
                self.cmd_vel_pub.publish(cmd)
                self.publish_status(f"State: TURN | Hdg Err: {math.degrees(heading_error):.1f}°")
            else:
                self.state = "DRIVING_BACK"
                self.send_stop_command()
                self.get_logger().info("✅ 180° Turn Complete. State -> DRIVING_BACK")

        elif self.state == "DRIVING_BACK":
            distance_to_start = abs(self.current_odom_x - self.start_pos_odom)
            if distance_to_start > self.distance_tolerance_m:
                self.drive_straight()
                self.publish_status(f"State: BACK (Odom) | Dist to Start: {distance_to_start:.2f}m")
            else:
                self.state = "IDLE"
                self.send_stop_command()
                final_pos_gps = (self.current_lat, self.current_lon)
                closing_error = self.gps_to_meters(self.start_pos_gps[0], self.start_pos_gps[1], final_pos_gps[0], final_pos_gps[1])
                self.get_logger().info(f"🏁 TEST COMPLETE | Final GPS Closing Error: {closing_error*100:.1f} cm")
                self.results_pub.publish(String(data=f"END | Lat: {final_pos_gps[0]:.8f}, Lon: {final_pos_gps[1]:.8f} | GPS Closing Error: {closing_error:.3f}m"))

    def drive_straight(self):
        heading_error = self.normalize_angle(self.target_heading - self.current_yaw)
        cmd = Twist()
        cmd.linear.x = self.drive_speed
        cmd.angular.z = self.heading_kp * heading_error
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self, status_text):
        accuracy_cm = 0.0
        if self.position_covariance is not None and len(self.position_covariance) >= 5:
            accuracy_cm = math.sqrt(self.position_covariance[0] + self.position_covariance[4]) * 100
        self.status_pub.publish(String(data=f"{status_text} | Accuracy: {accuracy_cm:.1f}cm"))

    def send_stop_command(self): self.cmd_vel_pub.publish(Twist())

    def gps_to_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        dlon, dlat = lon2_rad - lon1_rad, lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = RTKTestNodeReturn()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 