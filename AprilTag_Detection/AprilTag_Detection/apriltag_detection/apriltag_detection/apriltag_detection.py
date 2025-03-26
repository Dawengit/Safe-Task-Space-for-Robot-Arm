import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import apriltag
import tf2_ros
import tf2_geometry_msgs
import transforms3d.quaternions  # Replacement for tf_transformations

class AprilTagSafetyZone(Node):
    def __init__(self):
        super().__init__('april_tag_safety_zone')
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit()

        # AprilTag detector
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.target_tag_ids = [0, 1, 2, 3]
        self.tag_positions = {}
        
        # Camera parameters
        self.tagsize = 0.035
        self.image_width = 1280
        self.image_height = 720
        self.fx = 1200
        self.fy = 1195
        self.cx = 640.5
        self.cy = 360.5
        self.dist_coeffs = [0.12, -0.25, 0.001, 0.003, 0.15]

        # ROS components
        self.image_pub = self.create_publisher(Image, 'camera_image', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'safety_zone', 10)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Safety zone parameters
        self.safety_zone_min = None
        self.safety_zone_max = None
        
        # Timer for camera callback
        self.timer = self.create_timer(1/30.0, self.camera_callback)
        
        # Timer for checking finger position (10Hz)
        self.check_timer = self.create_timer(0.1, self.check_end_effector_position)

    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Cannot read frame")
            return

        frame = cv2.resize(frame, (self.image_width, self.image_height))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        detections = self.detector.detect(gray)
        current_tags = set()

        for detection in detections:
            tag_id = detection.tag_id
            if tag_id not in self.target_tag_ids:
                continue
            
            current_tags.add(tag_id)
            
            # Draw bounding box
            corners = detection.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0,255,0), 2)
            cv2.putText(frame, str(tag_id), tuple(corners[0]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

            # Calculate position
            object_points = np.array([
                [-self.tagsize/2, -self.tagsize/2, 0],
                [ self.tagsize/2, -self.tagsize/2, 0],
                [ self.tagsize/2,  self.tagsize/2, 0],
                [-self.tagsize/2,  self.tagsize/2, 0]
            ])
            
            camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            
            success, rvec, tvec = cv2.solvePnP(
                object_points, 
                detection.corners.astype('double'), 
                camera_matrix, 
                np.zeros(4)
            )

            if success:
                self.tag_positions[tag_id] = tvec.flatten()
                self.publish_tf_transform(tag_id, rvec, tvec)

        # Remove undetected tags
        for tag_id in list(self.tag_positions.keys()):
            if tag_id not in current_tags:
                del self.tag_positions[tag_id]

        self.update_safety_zone()
        self.publish_camera_image(frame)
        cv2.imshow('AprilTag Detection', frame)
        cv2.waitKey(1)

    def publish_tf_transform(self, tag_id, rvec, tvec):
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        
        # Convert rotation matrix to quaternion using transforms3d
        quaternion = transforms3d.quaternions.mat2quat(rotation_matrix[:3, :3])
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_frame"
        transform.child_frame_id = f"tag_{tag_id}"
        
        transform.transform.translation.x = tvec[0][0]
        transform.transform.translation.y = tvec[1][0]
        transform.transform.translation.z = tvec[2][0]
        
        # transforms3d returns [w, x, y, z], ROS uses [x, y, z, w]
        transform.transform.rotation.x = quaternion[1]
        transform.transform.rotation.y = quaternion[2]
        transform.transform.rotation.z = quaternion[3]
        transform.transform.rotation.w = quaternion[0]
        
        self.tf_broadcaster.sendTransform(transform)

    def update_safety_zone(self):
        if len(self.tag_positions) < 4:
            self.get_logger().warn(f"Need 4 tags, detected {len(self.tag_positions)}")
            marker = Marker(action=Marker.DELETE)
            self.marker_pub.publish(marker)
            self.safety_zone_min = None
            self.safety_zone_max = None
            return

        expansion_margin = 0   # XY plane expansion (meters)
        z_margin = 1.0         # Z axis margin (meters)

        positions = np.array(list(self.tag_positions.values()))
        
        # Calculate safety zone boundaries
        min_coords = np.min(positions, axis=0)
        max_coords = np.max(positions, axis=0)
        
        # Apply margins
        min_coords[0] -= expansion_margin
        min_coords[1] -= expansion_margin
        max_coords[0] += expansion_margin
        max_coords[1] += expansion_margin
        
        min_z = np.min(positions[:, 2]) - z_margin
        max_z = np.max(positions[:, 2]) + z_margin
        
        # Store the safety zone boundaries
        self.safety_zone_min = np.array([min_coords[0], min_coords[1], min_z])
        self.safety_zone_max = np.array([max_coords[0], max_coords[1], max_z])
        
        # Publish visualization marker
        center = [
            (min_coords[0] + max_coords[0]) / 2,
            (min_coords[1] + max_coords[1]) / 2,
            (min_z + max_z) / 2
        ]
        
        dimensions = [
            max_coords[0] - min_coords[0],
            max_coords[1] - min_coords[1],
            max_z - min_z
        ]

        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]
        
        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

    def check_end_effector_position(self):
        """Check if endeffector is inside safety zone"""
        if self.safety_zone_min is None or self.safety_zone_max is None:
            return

        try:
            # Lookup transform from camera_frame to finger link
            transform = self.tf_buffer.lookup_transform(
                "camera_frame",  # webcam frame
                "robotiq_85_right_finger_tip_link",  # Change to correct endeffector if needed
                rclpy.time.Time())
            
            # Get finger position in camera frame
            finger_pos = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Check if finger is in safety zone
            inside = np.all(finger_pos >= self.safety_zone_min) and np.all(finger_pos <= self.safety_zone_max)
            
            if inside:
                self.get_logger().info("End effector is INSIDE safety zone", throttle_duration_sec=1.0)
            else:
                self.get_logger().warn("Attention! End effector is OUTSIDE safety zone!!!!", throttle_duration_sec=2.0)
           
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF error: {str(e)}")

    def publish_camera_image(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"
        self.image_pub.publish(ros_image)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSafetyZone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()