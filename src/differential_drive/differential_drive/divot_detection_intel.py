import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # Import String message type
from cv_bridge import CvBridge
import cv2
import supervision as sv
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np # Import numpy

class DivotDetectorNode(Node):
    def __init__(self):
        super().__init__('divot_detector_node')
        self.bridge = CvBridge()
        self.latest_depth_image = None # Add storage for depth image

        # --- NEW: Camera Intrinsics (for D435i) ---
        self.camera_fx = 615.0  # Focal length X
        self.camera_fy = 615.0  # Focal length Y
        self.camera_cx = 320.0  # Principal point X (for 640 width)
        self.camera_cy = 240.0  # Principal point Y (for 480 height)
        # ---

        # --- Model Initialization ---
        package_path = get_package_share_directory('differential_drive')
        model_path = os.path.join(package_path, '1600s_aug_100ep.pt')
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("✅ YOLO model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLO model: {e}")
            raise

        # --- Supervision Annotators ---
        self.box_annotator = sv.BoxAnnotator()
        self.mask_annotator = sv.MaskAnnotator(color_lookup=sv.ColorLookup.CLASS, opacity=0.5)
        self.label_annotator = sv.LabelAnnotator(text_position=sv.Position.TOP_LEFT, text_padding=3)

        # --- ROS2 Subscribers and Publishers ---
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Subscribing to the existing camera topic
            self.image_callback,
            10)
        
        # --- NEW: Subscriber for depth image ---
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)

        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/camera/divot_detection/image_raw', # Publishing the result
            10)
        
        # Publisher for detection details
        self.details_publisher = self.create_publisher(
            String,
            '/camera/divot_detection/details',
            10)
            
        self.get_logger().info("Divot Detector Node is running and waiting for images...")

    def depth_callback(self, msg: Image):
        """Store the latest depth image."""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Perform inference silently
        results = self.model(cv_image, verbose=False)[0]
        detections = sv.Detections.from_ultralytics(results)

        # --- Publish Detection Details ---
        # Only process detections if they exist and have masks
        if detections and detections.mask is not None:
            details_msg = String()
            details_list = []
            
            # Find the class ID for 'divot' to perform specific calculations
            class_names = self.model.names
            divot_class_id = -1
            try:
                divot_class_id = [k for k, v in class_names.items() if v == 'divot'][0]
            except IndexError:
                self.get_logger().warn("Model does not contain 'divot' class. Cannot calculate center.", throttle_duration_sec=5)

            for i in range(len(detections.class_id)):
                class_id = detections.class_id[i]
                class_name = class_names[class_id]
                confidence = detections.confidence[i]

                # --- NEW: Calculate Mask Center for 'divot' class ---
                if class_id == divot_class_id:
                    # Get the boolean mask for the current divot
                    mask = detections.mask[i]
                    
                    # Calculate center coordinates from the mask
                    y_coords, x_coords = np.where(mask)
                    if len(x_coords) > 0 and len(y_coords) > 0:
                        center_x = int(np.mean(x_coords))
                        center_y = int(np.mean(y_coords))
                        
                        # Create the new, detailed string format
                        details_list.append(f"class:{class_name},confidence:{confidence:.2f},center_x:{center_x},center_y:{center_y}")

            if details_list:
                details_msg.data = ";".join(details_list)
                self.details_publisher.publish(details_msg)

        # Annotate the frame with detections
        annotated_frame = self.mask_annotator.annotate(
            scene=cv_image.copy(),
            detections=detections
        )
        annotated_frame = self.box_annotator.annotate(
            scene=annotated_frame,
            detections=detections
        )
        annotated_frame = self.label_annotator.annotate(
            scene=annotated_frame,
            detections=detections
        )

        # --- NEW: Visualize the calculated center point for divots ---
        if detections and detections.mask is not None:
            # First, draw the frame center crosshairs for reference
            frame_height, frame_width, _ = annotated_frame.shape
            frame_center_x = frame_width // 2
            frame_center_y = frame_height // 2
            cv2.line(annotated_frame, (0, frame_center_y), (frame_width, frame_center_y), (0, 255, 0), 1)
            cv2.line(annotated_frame, (frame_center_x, 0), (frame_center_x, frame_height), (0, 255, 0), 1)
            cv2.circle(annotated_frame, (frame_center_x, frame_center_y), 3, (255, 0, 255), -1) # Magenta dot for center

            # Re-iterate through detections to draw the center point on the annotated frame
            class_names = self.model.names
            try:
                divot_class_id = [k for k, v in class_names.items() if v == 'divot'][0]
                for i in range(len(detections.class_id)):
                    if detections.class_id[i] == divot_class_id:
                        mask = detections.mask[i]
                        y_coords, x_coords = np.where(mask)
                        if len(x_coords) > 0 and len(y_coords) > 0:
                            center_x = int(np.mean(x_coords))
                            center_y = int(np.mean(y_coords))
                            
                            # Draw a red circle at the calculated center
                            cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1) # -1 fills the circle
                            
                            # Draw horizontal and vertical lines for offset visualization
                            cv2.line(annotated_frame, (frame_center_x, center_y), (center_x, center_y), (0, 255, 0), 1) # Horizontal
                            cv2.line(annotated_frame, (center_x, frame_center_y), (center_x, center_y), (0, 255, 0), 1) # Vertical
                            
                            # --- NEW: Calculate and display real-world distance ---
                            if self.latest_depth_image is not None:
                                # Get depth at the center of the divot
                                depth_mm = self.latest_depth_image[center_y, center_x]
                                if depth_mm > 0:
                                    # Get bounding box for text positioning
                                    box = detections.xyxy[i]
                                    x1, y1 = int(box[0]), int(box[1])

                                    # Calculate and display depth distance
                                    depth_cm = depth_mm / 10.0
                                    distance_text = f"Dist: {depth_cm:.1f} cm"
                                    cv2.putText(annotated_frame, distance_text, (x1, y1 - 50), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                    
                                    # Calculate and display horizontal offset
                                    depth_m = depth_mm / 1000.0
                                    h_offset_m = ((center_x - self.camera_cx) * depth_m) / self.camera_fx
                                    h_offset_cm = h_offset_m * 100
                                    h_offset_text = f"H-Offset: {h_offset_cm:+.1f} cm"
                                    cv2.putText(annotated_frame, h_offset_text, (x1, y1 - 30), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                                    # Calculate and display vertical offset
                                    v_offset_m = ((center_y - self.camera_cy) * depth_m) / self.camera_fy
                                    v_offset_cm = v_offset_m * 100
                                    v_offset_text = f"V-Offset: {v_offset_cm:+.1f} cm"
                                    cv2.putText(annotated_frame, v_offset_text, (x1, y1 - 10), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                    
                                    # Display the full driving distance calculation for clarity
                                    total_drive_dist = depth_m + 0.90
                                    drive_dist_text = f"Drive Dist: {depth_m:.2f}m + 0.90m = {total_drive_dist:.2f}m"
                                    
                                    # Position text at the bottom of the bounding box
                                    text_size, _ = cv2.getTextSize(drive_dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                    text_x = x1
                                    text_y = int(box[3]) + 20 # y2 coordinate + padding
                                    
                                    cv2.putText(annotated_frame, drive_dist_text, (text_x, text_y), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2) # Cyan color

            except IndexError:
                pass # 'divot' class not in model, do nothing.
        # --- End Visualization ---

        try:
            # Convert annotated frame back to ROS Image message and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            annotated_msg.header = msg.header # Preserve the original timestamp and frame_id
            self.annotated_image_publisher.publish(annotated_msg)
            
            # --- Display the annotated frame for testing ---
            cv2.imshow("Divot Detection", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Could not publish or display annotated image: {e}")

def main(args=None):
    rclpy.init(args=args)
    divot_detector_node = DivotDetectorNode()
    try:
        rclpy.spin(divot_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        divot_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()