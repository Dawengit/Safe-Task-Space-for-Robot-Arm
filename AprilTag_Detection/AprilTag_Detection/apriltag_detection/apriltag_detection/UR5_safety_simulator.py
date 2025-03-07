import rclpy
from rclpy.node import Node
import numpy as np
import random
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class UR5SafetySimulator(Node):

    #----------------------------------------------------
    def __init__(self):
        super().__init__('ur5_safety_simulator')
        
        # initial_position:
        self.declare_parameters(
            namespace='',
            parameters=[
                ('simulation_rate', 30.0),
                ('safety_check_rate', 10.0),
                ('initial_position', [0.01, 0.02, 0.01]),
                ('movement_range', 1.0),
                ('safety_margin', 0.0)
            ]
        )
        
        # ----------------------------------------------------
        # init tf2_ros
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # state variables
        self.current_pose = np.array(self.get_parameter('initial_position').value)
        self.safety_zone = None
        self.safety_status = False
        
        # publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'tool_pose', 10)
        self.status_pub = self.create_publisher(Bool, 'safety_status', 10)
        self.marker_pub = self.create_publisher(Marker, 'safety_visualization', 10)
        
        # subscribers
        self.create_subscription(Marker, 'safety_zone', self.safety_zone_callback, 10)
        
        # timers
        self.create_timer(1.0/self.get_parameter('simulation_rate').value, 
                         self.simulation_callback)
        self.create_timer(1.0/self.get_parameter('safety_check_rate').value,
                         self.safety_check_callback)

    #----------------------------------------------------
    def safety_zone_callback(self, msg):
        """update safety zone"""
        '''arguments=[
                '1.0', '0.5', '2.0',  # X, Y, Z differiential
                '0', '0', '0',        # X, Y, Z rotation
                'ur5_base_link',      # father frame
                'camera_frame'        # son frame 
                # for this case, I use the camera_frame as the son frame
            ],
            name='camera_tf_publisher'''
        # 
        
        if msg.type == Marker.CUBE:
            self.safety_zone = {
                'center': np.array([msg.pose.position.x, 
                                  msg.pose.position.y,
                                  msg.pose.position.z]),
                'dimensions': np.array([msg.scale.x/2, 
                                      msg.scale.y/2,
                                      msg.scale.z/2])
            }

    #----------------------------------------------------
    def simulation_callback(self):
        """generate simulated robot movement"""
        # generate random movement
        self.current_pose += np.random.uniform(
            -self.get_parameter('movement_range').value/20,
            self.get_parameter('movement_range').value/20,
            3
        )
        self.current_pose += np.array([-0.0001, -0.0001, 0.0001])
        # self.current_pose = np.array([0.0, -0.1, 0.1])
        
        # publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "ur5_base_link"
        pose_msg.pose.position.x = self.current_pose[0]
        pose_msg.pose.position.y = self.current_pose[1]
        pose_msg.pose.position.z = self.current_pose[2]
        self.pose_pub.publish(pose_msg)
        
        # publish tf
        self.publish_robot_tf()

    #----------------------------------------------------
    def safety_check_callback(self):
        """Execute safety check"""
        if self.safety_zone is None:
            return
            
        try:
            # coordinate transform
            transform = self.tf_buffer.lookup_transform(
                'camera_frame',
                'ur5_base_link',
                rclpy.time.Time())
            
            # coordinate transform
            point = PointStamped()
            point.header.frame_id = "ur5_base_link"

            point.point.x = self.current_pose[0]
            point.point.y = self.current_pose[1]
            point.point.z = self.current_pose[2]
            transformed_point = do_transform_point(point, transform)
            
            # safety check
            self.safety_status = self.check_safety(transformed_point)
            
            # publish safety status
            self.publish_safety_status(transformed_point)
            self.publish_end_effector_marker(transformed_point)
            
        except Exception as e:
            self.get_logger().error(f'Coordinate Transform Failed: {str(e)}')
    
    #----------------------------------------------------
    def check_safety(self, point):
        """safety check algorithm"""
        offset = np.array([
            point.point.x - self.safety_zone['center'][0],
            point.point.y - self.safety_zone['center'][1],
            point.point.z - self.safety_zone['center'][2]
        ])
        
        return all(np.abs(offset) < (self.safety_zone['dimensions'] - 
                                   self.get_parameter('safety_margin').value))

    # def check_safety(self, point):
    #     offset = np.array([
    #         point.point.x - self.safety_zone['center'][0],
    #         point.point.y - self.safety_zone['center'][1],
    #         point.point.z - self.safety_zone['center'][2]
    #     ])
    #     # safety margin
    #     margin = max(self.get_parameter('safety_margin').value, 0)
    #     return all(np.abs(offset) < (self.safety_zone['half_dimensions'] - margin))

    #----------------------------------------------------
    def publish_safety_status(self, point):
        """publish safety status"""
        status_msg = Bool()
        status_msg.data = self.safety_status
        self.status_pub.publish(status_msg)
        
        log_msg = f"Position: ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f}) | "
        log_msg += "SAFE" if self.safety_status else "DANGER!!"
        self.get_logger().info(log_msg, throttle_duration_sec=1)

    #----------------------------------------------------
    def publish_end_effector_marker(self, point):
        """publish end effector visualization marker"""
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "end_effector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0  #  use blue to indicate the end

        # create text marker
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "coordinates"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = f"({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})"
        text_marker.pose.position.x = point.point.x
        text_marker.pose.position.y = point.point.y
        text_marker.pose.position.z = point.point.z + 0.15
        text_marker.scale.z = 0.1
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0

        self.marker_pub.publish(marker)
        self.marker_pub.publish(text_marker)

    #----------------------------------------------------
    def publish_robot_tf(self):
        """puiblish robot tf"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "ur5_base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UR5SafetySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()