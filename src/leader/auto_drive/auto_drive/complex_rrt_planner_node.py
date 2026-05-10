import math
import random

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from auto_drive.complex_rrt_core import RrtPlanner
from auto_drive.complex_rrt_core import RrtPlannerConfig


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def make_point(x, y, z=0.0):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def transform_xy(x, y, transform):
    translation = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        translation.x + cos_yaw * x - sin_yaw * y,
        translation.y + sin_yaw * x + cos_yaw * y,
    )


class ComplexRrtPlannerNode(Node):
    def __init__(self):
        super().__init__('complex_rrt_planner_node')

        self.planning_frame = self.declare_parameter(
            'planning_frame', 'vehicle_ref'
        ).value
        self.cone_pose_array_topic = self.declare_parameter(  #cone 토픽명
            'cone_pose_array_topic', '/complex/cones'
        ).value
        self.cone_marker_topic = self.declare_parameter(
            'cone_marker_topic', '/detected_objects_center'
        ).value
        self.target_topic = self.declare_parameter(
            'target_topic', '/complex/rrt_target'
        ).value
        self.predicted_obstacle_topic = self.declare_parameter(
            'predicted_obstacle_topic', '/complex/predicted_obstacles'
        ).value
        self.local_path_topic = self.declare_parameter(
            'local_path_topic', '/complex/local_path'
        ).value
        self.status_topic = self.declare_parameter(
            'status_topic', '/complex/rrt_status'
        ).value
        self.tree_marker_topic = self.declare_parameter(
            'tree_marker_topic', '/complex/rrt_tree'
        ).value
        self.obstacle_marker_topic = self.declare_parameter(
            'obstacle_marker_topic', '/complex/rrt_obstacles'
        ).value
        self.path_marker_topic = self.declare_parameter(
            'path_marker_topic', '/complex/local_path_marker'
        ).value

        self.plan_rate_hz = float(
            self.declare_parameter('plan_rate_hz', 8.0).value
        )
        self.cone_timeout_sec = float(
            self.declare_parameter('cone_timeout_sec', 0.7).value
        )
        self.target_timeout_sec = float(
            self.declare_parameter('target_timeout_sec', 0.7).value
        )
        random_seed = int(self.declare_parameter('random_seed', 0).value)
        self.rng = random.Random(random_seed) if random_seed else random.Random()

        config = RrtPlannerConfig(
            plan_distance=float(
                self.declare_parameter('plan_distance', 5.0).value
            ),
            expand_distance=float(
                self.declare_parameter('expand_distance', 0.7).value
            ),
            turn_angle_deg=float(
                self.declare_parameter('turn_angle_deg', 20.0).value
            ),
            max_iterations=int(
                self.declare_parameter('max_iterations', 700).value
            ),
            goal_tolerance=float(
                self.declare_parameter('goal_tolerance', 0.8).value
            ),
            target_sample_probability=float(
                self.declare_parameter(
                    'target_sample_probability', 0.75
                ).value
            ),
            target_sample_radius=float(
                self.declare_parameter('target_sample_radius', 2.5).value
            ),
            random_sample_x_min=float(
                self.declare_parameter('random_sample_x_min', 0.0).value
            ),
            random_sample_x_max=float(
                self.declare_parameter('random_sample_x_max', 7.0).value
            ),
            random_sample_y_abs=float(
                self.declare_parameter('random_sample_y_abs', 4.0).value
            ),
            cone_radius=float(self.declare_parameter('cone_radius', 0.8).value),
            predicted_obstacle_radius=float(
                self.declare_parameter('predicted_obstacle_radius', 0.6).value
            ),
            min_obstacle_x=float(
                self.declare_parameter('min_obstacle_x', -0.5).value
            ),
            max_obstacle_distance=float(
                self.declare_parameter('max_obstacle_distance', 12.0).value
            ),
            max_obstacle_abs_y=float(
                self.declare_parameter('max_obstacle_abs_y', 6.0).value
            ),
            min_target_x=float(
                self.declare_parameter('min_target_x', 0.8).value
            ),
            path_resolution=float(
                self.declare_parameter('path_resolution', 0.25).value
            ),
            allow_direct_path=bool(
                self.declare_parameter('allow_direct_path', True).value
            ),
            require_cones=bool(
                self.declare_parameter('require_cones', True).value
            ),
        )
        self.planner = RrtPlanner(config)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cones = []
        self.latest_cone_stamp_sec = None
        self.latest_target = None
        self.latest_target_stamp_sec = None
        self.latest_predicted_obstacles = []
        self.latest_predicted_stamp_sec = None
        self.last_tf_warn_ns = 0
        self.last_status = None

        self.create_subscription(
            PoseArray, self.cone_pose_array_topic, self.cone_pose_callback, 10
        )
        self.create_subscription(
            MarkerArray, self.cone_marker_topic, self.cone_marker_callback, 10
        )
        self.create_subscription(
            PointStamped, self.target_topic, self.target_callback, 10
        )
        self.create_subscription(
            MarkerArray,
            self.predicted_obstacle_topic,
            self.predicted_obstacle_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            Path, self.local_path_topic, TRANSIENT_QOS
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.tree_pub = self.create_publisher(
            MarkerArray, self.tree_marker_topic, 10
        )
        self.obstacle_pub = self.create_publisher(
            MarkerArray, self.obstacle_marker_topic, 10
        )
        self.path_marker_pub = self.create_publisher(
            Marker, self.path_marker_topic, 10
        )

        self.timer = self.create_timer(
            1.0 / max(self.plan_rate_hz, 1e-3), self.timer_callback
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cone_pose_callback(self, msg: PoseArray):
        cones = []
        for pose in msg.poses:
            point = self.transform_point(
                pose.position.x, pose.position.y, msg.header.frame_id
            )
            if point is not None:
                cones.append(point)
        self.latest_cones = cones
        self.latest_cone_stamp_sec = self.now_sec()

    def cone_marker_callback(self, msg: MarkerArray):
        cones = []
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            point = self.transform_point(
                marker.pose.position.x,
                marker.pose.position.y,
                marker.header.frame_id,
            )
            if point is not None:
                cones.append(point)
        self.latest_cones = cones
        self.latest_cone_stamp_sec = self.now_sec()

    def target_callback(self, msg: PointStamped):
        point = self.transform_point(
            msg.point.x, msg.point.y, msg.header.frame_id
        )
        if point is None:
            return
        self.latest_target = point
        self.latest_target_stamp_sec = self.now_sec()

    def predicted_obstacle_callback(self, msg: MarkerArray):
        obstacles = []
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            point = self.transform_point(
                marker.pose.position.x,
                marker.pose.position.y,
                marker.header.frame_id,
            )
            if point is None:
                continue
            radius = max(marker.scale.x, marker.scale.y, marker.scale.z) * 0.5
            if radius <= 0.0:
                radius = self.planner.config.predicted_obstacle_radius
            obstacles.append((point[0], point[1], radius))
        self.latest_predicted_obstacles = obstacles
        self.latest_predicted_stamp_sec = self.now_sec()

    def transform_point(self, x, y, source_frame):
        source_frame = source_frame or self.planning_frame
        if source_frame == self.planning_frame:
            return (float(x), float(y))

        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.03),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    'TF lookup failed '
                    f'({self.planning_frame} <- {source_frame}): {exc}'
                )
                self.last_tf_warn_ns = now_ns
            return None

        return transform_xy(x, y, transform)

    def timer_callback(self):
        now_sec = self.now_sec()
        missing_reason = self.missing_input_reason(now_sec)
        if missing_reason is not None:
            self.publish_empty_path(missing_reason)
            return

        result = self.planner.plan(
            self.latest_cones,
            self.latest_target,
            predicted_obstacles=self.current_predicted_obstacles(now_sec),
            rng=self.rng,
        )
        self.publish_path(result.path_points)
        self.publish_tree(result.tree_edges)
        self.publish_obstacles(result.obstacles)
        self.publish_path_marker(result.path_points)
        self.publish_status(result.reason)

    def missing_input_reason(self, now_sec):
        if self.latest_target is None:
            return 'missing_target'
        if now_sec - self.latest_target_stamp_sec > self.target_timeout_sec:
            return 'stale_target'
        if self.planner.config.require_cones:
            if self.latest_cone_stamp_sec is None:
                return 'missing_cones'
            if now_sec - self.latest_cone_stamp_sec > self.cone_timeout_sec:
                return 'stale_cones'
        return None

    def current_predicted_obstacles(self, now_sec):
        if self.latest_predicted_stamp_sec is None:
            return []
        if now_sec - self.latest_predicted_stamp_sec > self.cone_timeout_sec:
            return []
        return self.latest_predicted_obstacles

    def publish_empty_path(self, reason):
        self.publish_path([])
        self.publish_tree([])
        self.publish_obstacles([])
        self.publish_path_marker([])
        self.publish_status(reason)

    def publish_path(self, points):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.planning_frame
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = make_point(x, y, 0.0)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def publish_tree(self, tree_edges):
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.planning_frame
        marker.ns = 'complex_rrt_tree'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        marker.color.a = 0.45
        marker.color.r = 0.1
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        for start, end in tree_edges:
            marker.points.append(make_point(start[0], start[1], 0.02))
            marker.points.append(make_point(end[0], end[1], 0.02))
        marker_array.markers.append(marker)
        self.tree_pub.publish(marker_array)

    def publish_obstacles(self, obstacles):
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        stamp = self.get_clock().now().to_msg()
        for index, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.planning_frame
            marker.ns = 'complex_rrt_obstacles'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = make_point(obstacle.x, obstacle.y, 0.0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = obstacle.radius * 2.0
            marker.scale.y = obstacle.radius * 2.0
            marker.scale.z = 0.1
            marker.color.a = 0.28
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker_array.markers.append(marker)
        self.obstacle_pub.publish(marker_array)

    def publish_path_marker(self, points):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.planning_frame
        marker.ns = 'complex_local_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.color.a = 0.9
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.pose.orientation.w = 1.0
        for x, y in points:
            marker.points.append(make_point(x, y, 0.04))
        self.path_marker_pub.publish(marker)

    def publish_status(self, status):
        if status != self.last_status:
            self.get_logger().info(f'Complex RRT status -> {status}')
            self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ComplexRrtPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
