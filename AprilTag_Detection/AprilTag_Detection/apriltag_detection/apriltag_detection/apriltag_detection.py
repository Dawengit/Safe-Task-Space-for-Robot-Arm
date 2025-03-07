import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import apriltag
import tf2_ros
import tf_transformations


# Class AprilTagSafetyZone
"""This is the main class for the AprilTag safety zone node. 
It is responsible for detecting AprilTags in the camera feed, calculating their positions, 
and publishing the safety zone as a visualization marker(task space).
"""
class AprilTagSafetyZone(Node):
    def __init__(self):
        super().__init__('april_tag_safety_zone')
        self.bridge = CvBridge()
        # ---------------------------------------------------
        # self.end_effector_sub = self.create_subscription(
        #     PoseStamped,
        #     'tool_pose',
        #     self.handle_end_effector_pose,
        #     10)

        # # ---------------------------------------------------
        
        # initialize the camera
        # self.cap = cv2.VideoCapture(0)
        # 2 is the camera number for the Logitech C920x(webcam)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("cannot open camera")
            exit()

        # --------------------------------------------------------------------------
        # AprilTag detector
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.target_tag_ids = [0, 1, 2, 3]  # Tag ID to detect
        self.tag_positions = {}  # Store the position of each tag
        
        # ---------------------------------------------------
        # C920 camera parameters （C922 pro webcam）
        # After calibration, the camera parameters are as follows
        # self.tagsize = 0.035  # Tag actual size (m)
        self.tagsize = 0.035  # Tag actual size (m)
        self.image_width = 1280  # Recommended to use the native resolution
        self.image_height = 720
        self.fx = 1200       # Focal length x (calibrated value)
        self.fy = 1195       # Focal length y
        self.cx = 640.5        # Optical center x
        self.cy = 360.5       # Optical center y
        # ---------------------------------------------------
        self.dist_coeffs = [0.12, -0.25, 0.001, 0.003, 0.15]

        # C922 camera parameters
        # self.tagsize = 0.035  # 
        # self.image_width = 1920  # 
        # self.image_height = 1080
        # self.fx = 1200       # Focus length x
        # self.fy = 1195       # Focus length y
        # self.cx = 960.5        # Optical center x
        # self.cy = 540.5       # Optical center y
        # self.dist_coeffs = [0.12, -0.25, 0.001, 0.003, 0.15]

        # ---------------------------------------------------------
        # Ros2 components initialization
        self.image_pub = self.create_publisher(Image, 'camera_image', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'safety_zone', 10)
        
        # timer for camera callback(30HZ)
        self.timer = self.create_timer(1/30.0, self.camera_callback)

    # ---------------------------------------------------

    def camera_callback(self):
        # Read camera frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("cannot read frame")
            return

        # --------------------------------------------------------
        # Get timestamp 
        timestamp = self.get_clock().now().to_msg()  # Get the current time
        

        #---------------------------------------------------
        # picture processing
        frame = cv2.resize(frame, (self.image_width, self.image_height))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        

        #---------------------------------------------------
        # AprilTags
        detections = self.detector.detect(gray)
        current_tags = set()

        for detection in detections:
            tag_id = detection.tag_id
            if tag_id not in self.target_tag_ids:
                continue
            
            # record the currently detected tags
            current_tags.add(tag_id)
            
            # create a bounding box around the tag
            corners = detection.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0,255,0), 2)
            cv2.putText(frame, str(tag_id), tuple(corners[0]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

            # ---------------------------------------------------
            # calculate the position
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
                # save the position
                self.tag_positions[tag_id] = tvec.flatten()
    
                # publish the TF transform
                self.publish_tf_transform(tag_id, rvec, tvec)

        # remove the tags that are not detected
        for tag_id in list(self.tag_positions.keys()):
            if tag_id not in current_tags:
                del self.tag_positions[tag_id]

        # update the safety zone
        self.update_safety_zone()

        # publish the camera image
        self.publish_camera_image(frame)

        # show the frame
        cv2.imshow('AprilTag Detection', frame)
        cv2.waitKey(1)

#----------------------------------------------------------

    def publish_tf_transform(self, tag_id, rvec, tvec):

        # transform the rotation vector to quaternion
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix)
        
        # create the TF transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_frame"
        transform.child_frame_id = f"tag_{tag_id}"
        
        transform.transform.translation.x = tvec[0][0]
        transform.transform.translation.y = tvec[1][0]
        transform.transform.translation.z = tvec[2][0]
        
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(transform)

#-----------------------------------------------------------------

    def update_safety_zone(self):
        if len(self.tag_positions) < 4:
            self.get_logger().warn(f"At least  4 tags need to be detect, {len(self.tag_positions)} detected")
            # delete the old safety zone
            marker = Marker(action=Marker.DELETE)
            self.marker_pub.publish(marker)
            return

        # ------------------------- parameters -------------------------
        self.expansion_margin = 0   # XY plane expansion margin（unit: meters）
        self.z_margin = 1.0         # Z axis height expansion margin (unit: meters)

        # ------------------------- Get the position  -------------------------
        positions = np.array(list(self.tag_positions.values()))
        
        # ------------------------- safe space calculation-------------------------
        # 1. calculate the minimum and maximum coordinates
        min_coords = np.min(positions, axis=0)
        max_coords = np.max(positions, axis=0)
        
        # 2. Apply XY plane expansion
        min_coords[0] -= self.expansion_margin  # X minimum value expansion
        min_coords[1] -= self.expansion_margin  # Y minimum value expansion
        max_coords[0] += self.expansion_margin  # X maximum value expansion
        max_coords[1] += self.expansion_margin  # Y maximum value expansion
        
        # 3. dynamic calculation of the height of the safety zone
        min_z = np.min(positions[:, 2]) - self.z_margin  # Bottom height
        max_z = np.max(positions[:, 2]) + self.z_margin  # top height
        
        # ------------------------- central part and dimension -------------------------
        center = [
            (min_coords[0] + max_coords[0]) / 2,  # X center
            (min_coords[1] + max_coords[1]) / 2,  # Y center
            (min_z + max_z) / 2                   # Z center (dynamic adjustment)
        ]
        
        dimensions = [
            max_coords[0] - min_coords[0],  # X length
            max_coords[1] - min_coords[1],  # Y width
            max_z - min_z                   # Z height
        ]

        # ------------------------- Visualize Marker -------------------------
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # set the pose
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = dimensions[0]  # x axis length
        marker.scale.y = dimensions[1]  # y axis width
        marker.scale.z = dimensions[2]  # z axis height
        
        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

#--------------------------------------------------- 

    def publish_camera_image(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"
        self.image_pub.publish(ros_image)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

#---------------------------------------------------
# Main function
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
