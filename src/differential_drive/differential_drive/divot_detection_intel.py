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

class DivotDetectorNode(Node):
    def __init__(self):
        super().__init__('divot_detector_node')
        self.bridge = CvBridge()

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
        if detections and detections.class_id is not None and detections.confidence is not None:
            details_msg = String()
            details_list = []
            for i in range(len(detections.class_id)):
                class_name = self.model.names[detections.class_id[i]]
                confidence = detections.confidence[i]
                details_list.append(f"Detection: {class_name}, Confidence: {confidence:.2f}")
            details_msg.data = "; ".join(details_list)
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