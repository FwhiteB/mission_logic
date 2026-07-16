import json
from pathlib import Path

import rclpy
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from mission_logic_msgs.msg import SensorMsg
from visualization_msgs.msg import Marker

from mission_logic.geometry import Point3D
from mission_logic.old_field_sim import BiotSavartFieldModel, Pipeline, SimpleFieldModel

from mission_logic.models import RobotPose
from mission_logic.mission_node import quaternion_to_yaw


class MagneticFieldNode(Node):
    def __init__(self):
        super().__init__('magnetic_field_node')

        self.declare_parameter('pipeline_config_file', '')
        self.declare_parameter('model_type', 'simple')
        self.declare_parameter(
            'pipeline_points',
            [-1000.0, 0.0, -1.0, 1000.0, 0.0, -1.0],
        )
        self.declare_parameter('pipeline_name', 'straight_wire')
        self.declare_parameter('source_strength', 1.0)
        self.declare_parameter('background', 0.0)
        self.declare_parameter('distance_floor', 0.1)
        self.declare_parameter('attenuation_power', 2.0)
        self.declare_parameter('current_scale', 100.0)
        self.declare_parameter('signal_scale', 2000000.0)
        self.declare_parameter('singularity_floor', 0.05)
        self.declare_parameter('twoarrows_threshold', 0.98894)
        self.declare_parameter('compass_depth_and_currunt_and_onearrow_threshold', 0.95783)
        self.declare_parameter('heading_degrees_threshold', 32.0)
        self.declare_parameter('receiver_robot_dx', 1.0)
        self.declare_parameter('receiver_robot_dy', 1.0)
        self.declare_parameter('receiver_robot_same_direction', True)

        self.receiver_robot_dx = self.get_parameter('receiver_robot_dx').value
        self.receiver_robot_dy = self.get_parameter('receiver_robot_dy').value
        self.receiver_robot_same_direction = self.get_parameter('receiver_robot_same_direction').value

        self.config = self._load_config()
        self.field_model = self._build_field_model()
        self.twoarrows_threshold = self.get_parameter('twoarrows_threshold').value
        self.compass_depth_and_currunt_and_onearrow_threshold = self.get_parameter(
            'compass_depth_and_currunt_and_onearrow_threshold'
        ).value
        self.heading_degrees_threshold = self.get_parameter('heading_degrees_threshold').value


        self.state_subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.state_estimation_callback,
            10,
        )
        self.magnetic_field_publisher = self.create_publisher(
            SensorMsg,
            '/magnetic_field',
            10,
        )
        self.receiver_marker_publisher = self.create_publisher(Marker, '/receiver_marker', 5)
        self.pipeline_marker_publisher = self.create_publisher(Marker, '/pipeline_marker', 1)
        self.pipeline_fzx_marker_publisher = self.create_publisher(Marker, '/pipeline_fzx_marker', 1)
        self.create_timer(1.0, self.publish_pipeline_marker)

    def state_estimation_callback(self, odometry_msg):
        orientation = odometry_msg.pose.pose.orientation
        robot_yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        yaw = robot_yaw if self.receiver_robot_same_direction else (robot_yaw + math.pi) % (2 * math.pi) # 探测器的方向
        position = RobotPose(
            odometry_msg.pose.pose.position.x,
            odometry_msg.pose.pose.position.y,
            odometry_msg.pose.pose.position.z,
            robot_yaw
        )
        rr_distance = math.hypot(self.receiver_robot_dx, self.receiver_robot_dy)
        rr_yaw = math.atan2(self.receiver_robot_dy, self.receiver_robot_dx)
        Delta_x = rr_distance * math.cos(robot_yaw + rr_yaw)
        Delta_y = rr_distance * math.sin(robot_yaw + rr_yaw)
        receiver_x = position.x + Delta_x
        receiver_y = position.y + Delta_y
        position_point = Point3D(receiver_x, receiver_y, position.z) # 这里不太严谨地认为探测器的高度和机器人一样，后续可以改进
        measurement = self.field_model.sample(position_point)
        # field_vector = self._field_vector(measurement, position)
        field_vector = Point3D(measurement.magnetic_x, measurement.magnetic_y, 0.0)

        magnetic_field_msg = SensorMsg()
        magnetic_field_msg.header.stamp = odometry_msg.header.stamp
        magnetic_field_msg.header.frame_id = odometry_msg.header.frame_id
        magnetic_field_msg.magnetic_field.x = field_vector.x
        magnetic_field_msg.magnetic_field.y = field_vector.y
        magnetic_field_msg.magnetic_field.z = field_vector.z
        magnetic_field_msg.magnetic_field_covariance = [0.0] * 9
        # magnetic_field_msg.signal_strength = math.abs(field_vector.y)
        nearest_pipeline = measurement.nearest_pipeline
        if nearest_pipeline is not None:
            nearest_depth = measurement.true_depth
            nearest_distance = measurement.nearest_distance
            if nearest_depth is None or nearest_distance is None:
                self.get_logger().warn("nearest_depth or nearest_distance is None")
                return
            real_heading_degrees = (measurement.heading_degrees - math.degrees(yaw)) % 360
            if real_heading_degrees > 180:  # the compass is bidirectional (0~360 -> -90~90)
                real_heading_degrees -= 180
            if real_heading_degrees > 90:
                real_heading_degrees -= 180
            degree_jugde = abs(real_heading_degrees) < self.heading_degrees_threshold
            if ((nearest_depth / nearest_distance) >= self.twoarrows_threshold) and degree_jugde:
                magnetic_field_msg.left_arrow = True
                magnetic_field_msg.right_arrow = True
                magnetic_field_msg.depth_meters = nearest_depth
                magnetic_field_msg.current_milliamps = measurement.current_value
                magnetic_field_msg.pipeline_heading_degrees = real_heading_degrees
            elif ((nearest_depth / nearest_distance) >= self.compass_depth_and_currunt_and_onearrow_threshold) and degree_jugde:
                magnetic_field_msg.depth_meters = nearest_depth
                magnetic_field_msg.current_milliamps = measurement.current_value
                magnetic_field_msg.pipeline_heading_degrees = real_heading_degrees
                robot_orientation_vector = Point3D(math.cos(yaw), math.sin(yaw), 0)
                if measurement.nearest_point is None:
                    self.get_logger().warn("nearest_point is None")
                    return
                robot_relative_position_vector = Point3D(
                    receiver_x - measurement.nearest_point.x,
                    receiver_y - measurement.nearest_point.y,
                    0
                )
                cross_product = robot_orientation_vector.x * robot_relative_position_vector.y - robot_orientation_vector.y * robot_relative_position_vector.x
                if cross_product > 0:
                    magnetic_field_msg.left_arrow = True
                elif cross_product < 0:
                    magnetic_field_msg.right_arrow = True
            else: # note: may not be accurate, later we will confirm
                magnetic_field_msg.pipeline_heading_degrees = real_heading_degrees
                magnetic_field_msg.left_arrow = False
                magnetic_field_msg.right_arrow = False
            


        # convert to dashboard readings
        field_y = field_vector.y * math.cos(yaw) - field_vector.x * math.sin(yaw)
        field_x = field_vector.y * math.sin(yaw) + field_vector.x * math.cos(yaw)
        field = Point3D(field_x, field_y, 0.0)
        magnetic_field_msg.signal_strength = field.y if field.y > 0 else 0.0 # notice！: 这样修改会不会导致探测器朝向前后无法区分！？

        receiver_pose = RobotPose(receiver_x, receiver_y, position.z, yaw)
        self.publish_receiver_marker(receiver_pose=receiver_pose)
        self.magnetic_field_publisher.publish(magnetic_field_msg)

    def publish_pipeline_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'mission_logic'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        for pipeline in self.field_model.pipelines:
            for point in pipeline.points:
                marker_point = Point()
                marker_point.x = point.x
                marker_point.y = point.y
                marker_point.z = 1.0
                marker.points.append(marker_point)

        self.pipeline_marker_publisher.publish(marker)

    def publish_receiver_marker(self, receiver_pose):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'mission_logic'
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        arrow_length = 1.0
        z = receiver_pose.z + 0.4

        start = Point()
        start.x = receiver_pose.x
        start.y = receiver_pose.y
        start.z = z

        end = Point()
        end.x = receiver_pose.x + arrow_length * math.cos(receiver_pose.yaw)
        end.y = receiver_pose.y + arrow_length * math.sin(receiver_pose.yaw)
        end.z = z

        marker.points.append(start)
        marker.points.append(end)

        marker.scale.x = 0.08   # 箭杆直径
        marker.scale.y = 0.22   # 箭头直径
        marker.scale.z = 0.25   # 箭头长度

        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.receiver_marker_publisher.publish(marker)

    def publish_pipeline_fzx_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'fzx'
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points.append(Point(x=12.0, y=0.0, z=0.0))
        marker.points.append(Point(x=-12.0, y=24.0, z=0.0))
        self.pipeline_fzx_marker_publisher.publish(marker)

    def _load_config(self):
        config_path = str(self.get_parameter('pipeline_config_file').value)
        if not config_path:
            return {}

        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'pipeline_config_file does not exist: {path}')
        raw = json.loads(path.read_text(encoding='utf-8-sig'))
        if not isinstance(raw, dict):
            raise ValueError('Top-level pipeline config must be a JSON object.')
        return raw

    def _build_field_model(self):
        pipeline_config = self.config.get('pipeline', {})
        field_model_config = self.config.get('field_model', {})

        pipeline = Pipeline.from_tuples(
            points=self._pipeline_points(pipeline_config),
            source_strength=float(
                pipeline_config.get('source_strength', self.get_parameter('source_strength').value)
            ),
            name=str(pipeline_config.get('name', self.get_parameter('pipeline_name').value)),
        )
        model_type = str(
            field_model_config.get('type', self.get_parameter('model_type').value)
        ).lower()
        if model_type == 'simple':
            return SimpleFieldModel(
                [pipeline],
                background=float(
                    field_model_config.get('background', self.get_parameter('background').value)
                ),
                distance_floor=float(
                    field_model_config.get(
                        'distance_floor',
                        self.get_parameter('distance_floor').value,
                    )
                ),
                attenuation_power=float(
                    field_model_config.get(
                        'attenuation_power',
                        self.get_parameter('attenuation_power').value,
                    )
                ),
                current_scale=float(
                    field_model_config.get('current_scale', self.get_parameter('current_scale').value)
                ),
            )
        if model_type == 'biot_savart':
            return BiotSavartFieldModel(
                [pipeline],
                background=float(
                    field_model_config.get('background', self.get_parameter('background').value)
                ),
                current_scale=float(
                    field_model_config.get('current_scale', self.get_parameter('current_scale').value)
                ),
                signal_scale=float(
                    field_model_config.get('signal_scale', self.get_parameter('signal_scale').value)
                ),
                singularity_floor=float(
                    field_model_config.get(
                        'singularity_floor',
                        self.get_parameter('singularity_floor').value,
                    )
                ),
            )
        raise ValueError("model_type must be either 'simple' or 'biot_savart'.")

    def _pipeline_points(self, pipeline_config):
        if 'points' in pipeline_config:
            points = pipeline_config['points']
            if len(points) < 2:
                raise ValueError('pipeline.points must contain at least two xyz triples.')
            return [(float(point[0]), float(point[1]), float(point[2])) for point in points]

        flat_points = list(self.get_parameter('pipeline_points').value)
        if len(flat_points) < 6 or len(flat_points) % 3 != 0:
            raise ValueError('pipeline_points must contain at least two xyz triples.')

        points = []
        for index in range(0, len(flat_points), 3):
            points.append((
                float(flat_points[index]),
                float(flat_points[index + 1]),
                float(flat_points[index + 2]),
            ))
        return points

    def _field_vector(self, measurement, position):
        if measurement.magnetic_field is not None:
            return measurement.magnetic_field

        if measurement.nearest_point is None or measurement.nearest_pipeline is None:
            return Point3D(0.0, 0.0, measurement.signal_strength)

        pipeline = self._pipeline_by_name(measurement.nearest_pipeline)
        if pipeline is None:
            return Point3D(0.0, 0.0, measurement.signal_strength)

        nearest_point, tangent = pipeline.closest_projection(position)
        radius = position - nearest_point
        direction = tangent.cross(radius)
        direction_norm = direction.norm()
        if direction_norm == 0.0:
            return Point3D(0.0, 0.0, measurement.signal_strength)
        return direction.scale(measurement.signal_strength / direction_norm)

    def _pipeline_by_name(self, name):
        for pipeline in self.field_model.pipelines:
            if pipeline.name == name:
                return pipeline
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
