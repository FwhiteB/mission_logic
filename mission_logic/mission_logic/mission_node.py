import math
import copy
import json
import numpy as np
from typing import Optional


import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from mission_logic_msgs.msg import SensorMsg
from std_msgs.msg import Float32, String
from mission_logic.geometry import Point3D

from mission_logic.models import MissionLogEntry, MissionState, MoveResult, ReceiverReading, RobotPose, LineMeasurementPoint, LineMeasurementTrack, MemoryReadings, RobotState


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Robot:
    def __init__(
        self,
        node,
        speed_publisher,
        goal_frame='map',
        navigate_to_pose_action='/navigate_to_pose',
        speed = 1.0,
        receiver_robot_dx = 1.0,
        receiver_robot_dy = 1.0,
        receiver_robot_same_direction = True
    ):
        self._node = node
        self._speed_publisher = speed_publisher
        self._goal_frame = goal_frame
        self._speed = speed
        self.receiver_robot_dx = receiver_robot_dx
        self.receiver_robot_dy = receiver_robot_dy
        self.receiver_robot_same_direction = receiver_robot_same_direction
        self._navigate_to_pose_action = navigate_to_pose_action
        self._nav_action_client = ActionClient(
            self._node,
            NavigateToPose,
            self._navigate_to_pose_action,
        )

        self.pose: Optional[RobotPose] = None
        self.reading: Optional[ReceiverReading] = None
        self.active_goal: Optional[RobotPose] = None
        self._completed_goal: Optional[RobotPose] = None
        self._completed_goal_succeeded = False
        self._completed_goal_status: Optional[int] = None
        self._nav_goal_pending = False
        self._nav_goal_sequence = 0

    def update_pose(self, odometry_msg):
        orientation = odometry_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        position = odometry_msg.pose.pose.position
        self.pose = RobotPose(
            x=position.x,
            y=position.y,
            z=position.z,
            yaw=yaw,
        )

    def update_reading(self, magnetic_field_msg):

        stamp = magnetic_field_msg.header.stamp
        self.reading = ReceiverReading(
            signal_strength = magnetic_field_msg.signal_strength,
            depth = magnetic_field_msg.depth_meters,
            current = magnetic_field_msg.current_milliamps,
            pipeline_heading_degrees = magnetic_field_msg.pipeline_heading_degrees,
            signal_strength_percent = magnetic_field_msg.signal_strength_percent,
            left_arrow = magnetic_field_msg.left_arrow,  # notice: 指的是出现在左边的，指示机器向右的箭头，这表明机器本身在管线左边
            right_arrow = magnetic_field_msg.right_arrow,
            stamp_sec=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            frame_id=magnetic_field_msg.header.frame_id,
            magnetic_field = magnetic_field_msg.magnetic_field
        )

    def read(self):
        return self.pose, self.reading

    def robot_move_to(self, x, y, yaw):
        if self.pose is None:
            return None

        self.active_goal = RobotPose(
            x=x,
            y=y,
            z=self.pose.z,
            yaw=yaw,
        )
        target = self.active_goal
        # self._publish_speed()   # 空转的时候可以直接把activegoal设置为None，这是因为是根据ocr结果的到来决定下一步的。
        # self._send_active_goal()
        self.active_goal = None
        return MoveResult(target=target, reading=self.reading)

    def robot_tf_to_receiver(self, pose: RobotPose):
        if self.pose is None:
            return None
        
        receiver_yaw = pose.yaw if self.receiver_robot_same_direction else (pose.yaw + math.pi) % (2 * math.pi)
        rr_distance = math.hypot(self.receiver_robot_dx, self.receiver_robot_dy)
        rr_yaw = math.atan2(self.receiver_robot_dy, self.receiver_robot_dx)
        receiver_x = pose.x + rr_distance * math.cos(pose.yaw + rr_yaw)
        receiver_y = pose.y + rr_distance * math.sin(pose.yaw + rr_yaw)
        return RobotPose(x=receiver_x, y=receiver_y, z=pose.z, yaw=receiver_yaw)

    def receiver_move_to(self, x, y, yaw): # notice: 想加强的话甚至可以加一个角度差receiver_robot_dyaw。另外表述有点啰嗦，之后可以改一下
        if self.pose is None:
            return None

        yaw_offset = 0.0 if self.receiver_robot_same_direction else math.pi
        target_robot_yaw = (yaw - yaw_offset) % (2 * math.pi)

        offset_x = (
            self.receiver_robot_dx * math.cos(target_robot_yaw)
            - self.receiver_robot_dy * math.sin(target_robot_yaw)
        )
        offset_y = (
            self.receiver_robot_dx * math.sin(target_robot_yaw)
            + self.receiver_robot_dy * math.cos(target_robot_yaw)
        )

        target_robot_x = x - offset_x
        target_robot_y = y - offset_y

        return self.robot_move_to(target_robot_x, target_robot_y, target_robot_yaw)

    def _active_goal_to_pose_stamped(self):
        if self.active_goal is None:
            return None

        orientation_x, orientation_y, orientation_z, orientation_w = yaw_to_quaternion(
            self.active_goal.yaw
        )

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose_msg.header.frame_id = self._goal_frame
        goal_pose_msg.pose.position.x = self.active_goal.x
        goal_pose_msg.pose.position.y = self.active_goal.y
        goal_pose_msg.pose.position.z = self.active_goal.z
        goal_pose_msg.pose.orientation.x = orientation_x
        goal_pose_msg.pose.orientation.y = orientation_y
        goal_pose_msg.pose.orientation.z = orientation_z
        goal_pose_msg.pose.orientation.w = orientation_w

        return goal_pose_msg

    def _send_active_goal(self):
        goal_pose_msg = self._active_goal_to_pose_stamped()
        if goal_pose_msg is None:
            return

        self._nav_goal_sequence += 1
        goal_sequence = self._nav_goal_sequence
        self._completed_goal = None
        self._completed_goal_succeeded = False
        self._completed_goal_status = None
        self._nav_goal_pending = True

        if not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self._finish_navigation_goal(
                goal_sequence,
                succeeded=False,
                status=None,
                reason='Nav2 NavigateToPose action server is unavailable',
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose_msg
        goal_msg.behavior_tree = ''

        future = self._nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda response_future, sequence=goal_sequence:
            self._navigation_goal_response_callback(response_future, sequence)
        )

    def _navigation_goal_response_callback(self, future, goal_sequence):
        if goal_sequence != self._nav_goal_sequence:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS action transport
            self._finish_navigation_goal(
                goal_sequence,
                succeeded=False,
                status=None,
                reason='Nav2 goal request failed: %s' % exc,
            )
            return

        if not goal_handle.accepted:
            self._finish_navigation_goal(
                goal_sequence,
                succeeded=False,
                status=None,
                reason='Nav2 goal rejected',
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, sequence=goal_sequence:
            self._navigation_result_callback(result, sequence)
        )

    def _navigation_result_callback(self, future, goal_sequence):
        if goal_sequence != self._nav_goal_sequence:
            return

        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # pragma: no cover - depends on ROS action transport
            self._finish_navigation_goal(
                goal_sequence,
                succeeded=False,
                status=None,
                reason='Nav2 result failed: %s' % exc,
            )
            return
        self._finish_navigation_goal(
            goal_sequence,
            succeeded=(status == GoalStatus.STATUS_SUCCEEDED),
            status=status,
            reason='Nav2 goal finished with status=%s' % status,
        )

    def _finish_navigation_goal(self, goal_sequence, succeeded, status, reason):
        if goal_sequence != self._nav_goal_sequence:
            return

        self._completed_goal = self.active_goal
        self._completed_goal_succeeded = succeeded
        self._completed_goal_status = status
        self._nav_goal_pending = False
        self.active_goal = None
        if succeeded:
            self._node.get_logger().info(reason)
        else:
            self._node.get_logger().warning(reason)

    def has_pending_goal(self):
        return self._nav_goal_pending

    def consume_completed_goal(self):
        if self._completed_goal is None:
            return None

        completed_goal = self._completed_goal
        succeeded = self._completed_goal_succeeded
        status = self._completed_goal_status
        self._completed_goal = None
        self._completed_goal_succeeded = False
        self._completed_goal_status = None
        return completed_goal, succeeded, status

    def _publish_speed(self):
        speed_msg = Float32()
        speed_msg.data = float(self._speed)
        self._speed_publisher.publish(speed_msg)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('step_x', 1.0)
        self.declare_parameter('step_y', 0.0)
        self.declare_parameter('magnetic_y_gain', 0.5)
        self.declare_parameter('max_lateral_step', 1.0)
        self.declare_parameter('workspace_min_x', -20.0)
        self.declare_parameter('workspace_max_x', 20.0)
        self.declare_parameter('workspace_min_y', -20.0)
        self.declare_parameter('workspace_max_y', 20.0)
        self.declare_parameter('detect_threshold', 0.2)
        self.declare_parameter('loss_threshold', 0.08)
        self.declare_parameter('center_magnetic_z_threshold', 0.05)
        self.declare_parameter('search_probe_distance', 0.9)
        self.declare_parameter('centering_step', 0.4)
        self.declare_parameter('forward_step', 1.0)
        self.declare_parameter('reacquire_probe_offset', 0.4)
        self.declare_parameter('follow_heading_degrees', 0.0)
        self.declare_parameter('orientation_check_interval', 3)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('max_steps', 100)
        self.declare_parameter('close_threshold', 1.0)
        self.declare_parameter('repeat_count_threshold', 4)
        self.declare_parameter('memory_readings_capacity', 5)
        self.declare_parameter('receiver_robot_dx', 1.0)
        self.declare_parameter('receiver_robot_dy', 1.0) # 注意config里面有两个这个需要设置！
        self.declare_parameter('receiver_robot_same_direction', True)
        self.declare_parameter('debug_publish_period', 1.0)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('nav_recovery_wait_seconds', 1.0)
        self.declare_parameter('max_nav_recovery_attempts', 3)

        self.step_x = self.get_parameter('step_x').value
        self.step_y = self.get_parameter('step_y').value
        self.magnetic_y_gain = self.get_parameter('magnetic_y_gain').value
        self.max_lateral_step = self.get_parameter('max_lateral_step').value
        self.workspace_min_x = self.get_parameter('workspace_min_x').value
        self.workspace_max_x = self.get_parameter('workspace_max_x').value
        self.workspace_min_y = self.get_parameter('workspace_min_y').value
        self.workspace_max_y = self.get_parameter('workspace_max_y').value
        self.detect_threshold = self.get_parameter('detect_threshold').value
        self.loss_threshold = self.get_parameter('loss_threshold').value
        self.center_magnetic_z_threshold = self.get_parameter('center_magnetic_z_threshold').value  # notice: useless
        self.search_probe_distance = self.get_parameter('search_probe_distance').value
        self.centering_step = self.get_parameter('centering_step').value
        self.forward_step = self.get_parameter('forward_step').value
        self.reacquire_probe_offset = self.get_parameter('reacquire_probe_offset').value
        self.follow_heading_degrees = self.get_parameter('follow_heading_degrees').value  # useless
        self.orientation_check_interval = int(self.get_parameter('orientation_check_interval').value)
        self.max_steps = self.get_parameter('max_steps').value
        self.close_threshold = self.get_parameter('close_threshold').value
        self.repeat_count_threshold = self.get_parameter('repeat_count_threshold').value
        self.memory_readings_capacity = self.get_parameter('memory_readings_capacity').value
        self.receiver_robot_dx = self.get_parameter('receiver_robot_dx').value
        self.receiver_robot_dy = self.get_parameter('receiver_robot_dy').value
        self.receiver_robot_same_direction = self.get_parameter('receiver_robot_same_direction').value
        self.debug_publish_period = float(self.get_parameter('debug_publish_period').value)
        self.debug_mode = bool(self.get_parameter('debug_mode').value)
        self.nav_recovery_wait_seconds = float(self.get_parameter('nav_recovery_wait_seconds').value)
        self.max_nav_recovery_attempts = int(self.get_parameter('max_nav_recovery_attempts').value)
        # notice: follow_line step should > reacquire_step, for the correctness on conner

        self.speed_publisher = self.create_publisher(Float32, '/speed', 10)
        self.debug_publisher = self.create_publisher(String, '/mission_debug', 10)

        self.robot = Robot(
            node=self,
            speed_publisher=self.speed_publisher,
            goal_frame=self.get_parameter('goal_frame').value,
            navigate_to_pose_action=self.get_parameter('navigate_to_pose_action').value,
            speed=self.get_parameter('speed').value,
            receiver_robot_dx=self.receiver_robot_dx,   
            receiver_robot_dy=self.receiver_robot_dy,
            receiver_robot_same_direction=self.receiver_robot_same_direction
        )
        self.step_count = 0
        self.state = MissionState.SEARCH_PEAK
        self.follow_moves_since_center = 0
        self.line_confirmed = False
        self.log: list[MissionLogEntry] = []
        self.done = False
        self.phase = "normal"
        self.rotation_count = 0
        self.max_yaw = 0
        self.max_signal = float('-inf')
        self.min_yaw = 0
        self.min_signal = float('inf')
        self.wait_reading_after_stamp = 0.0
        self.need_fresh_reading = False
        self.fresh_reading_after_stamp = 0.0
        self.last_search_target_yaw = 0.0
        self.search_base_yaw = 0.0
        self.first_confirmed_measurement_pose: Optional[RobotPose] = None
        self.start_second_search = False
        self.start_second_search_turning = False
        self.standard = None
        self.repeat_count = 0
        self.memory_readings = MemoryReadings(memory_readings_capacity=self.memory_readings_capacity)
        self.conner_check_lock = False
        self.rotate_around_robot = self.receiver_robot_dx != 0 and self.receiver_robot_dy == 0
        self.confirmed_pipeline_yaw = None
        self.conners_number = []
        self.reverse_number = 0
        self.center_last_single_arrow = None
        self.center_last_single_pose: Optional[RobotPose] = None
        self.center_last_single_reading: Optional[ReceiverReading] = None
        self.virtual_measure_pose: Optional[RobotPose] = None
        self.virtual_measure_reading: Optional[ReceiverReading] = None
        self.follow_anchor_pose: Optional[RobotPose] = None
        self.nav_recovery_target: Optional[RobotPose] = None
        self.nav_recovery_failed_status: Optional[int] = None
        self.nav_recovery_ready_time_sec = 0.0
        self.nav_recovery_attempts = 0

        self.state_subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.state_estimation_callback,
            10,
        )
        self.magnetic_field_subscription = self.create_subscription(
            SensorMsg,
            '/magnetic_field',
            self.magnetic_field_callback,
            10,

        )
        # self.control_timer = self.create_timer(0.2, self._advance_state_machine)
        self.debug_timer = None
        if self.debug_publish_period > 0.0:
            self.debug_timer = self.create_timer(
                self.debug_publish_period,
                self.publish_debug_state,
            )
        self.measurement_track = LineMeasurementTrack()
        self.measurement_track_copy = LineMeasurementTrack()

    def state_estimation_callback(self, odometry_msg):
        self.robot.update_pose(odometry_msg)
        # self._advance_state_machine()

    def magnetic_field_callback(self, magnetic_field_msg):
        self.robot.update_reading(magnetic_field_msg)
        self._advance_state_machine()

    def publish_debug_state(self):
        pose = self.robot.pose
        reading = self.robot.reading
        active_goal = self.robot.active_goal
        receiver_pose = self.robot.robot_tf_to_receiver(pose) if pose is not None else None

        debug_state = {
            'state': self.state.value,
            'phase': self.phase,
            'done': self.done,
            'step_count': self.step_count,
            'follow_moves_since_center': self.follow_moves_since_center,
            'line_confirmed': self.line_confirmed,
            'rotation_count': self.rotation_count,
            'max_yaw_deg': self._degrees_or_none(self.max_yaw),
            'max_signal': self._finite_float_or_none(self.max_signal),
            'min_yaw_deg': self._degrees_or_none(self.min_yaw),
            'min_signal': self._finite_float_or_none(self.min_signal),
            'wait_reading_after_stamp': self._finite_float_or_none(self.wait_reading_after_stamp),
            'need_fresh_reading': self.need_fresh_reading,
            'fresh_reading_after_stamp': self._finite_float_or_none(self.fresh_reading_after_stamp),
            'last_search_target_yaw_deg': self._degrees_or_none(self.last_search_target_yaw),
            'search_base_yaw_deg': self._degrees_or_none(self.search_base_yaw),
            'start_second_search': self.start_second_search,
            'start_second_search_turning': self.start_second_search_turning,
            'repeat_count': self.repeat_count,
            'conner_check_lock': self.conner_check_lock,
            'rotate_around_robot': self.rotate_around_robot,
            'pose': self._pose_to_debug_dict(pose),
            'receiver_pose': self._pose_to_debug_dict(receiver_pose),
            'active_goal': self._pose_to_debug_dict(active_goal),
            'reading': self._reading_to_debug_dict(reading),
            'center_last_single_arrow': self.center_last_single_arrow,
            'center_last_single_pose': self._pose_to_debug_dict(self.center_last_single_pose),
            'virtual_measure_pose': self._pose_to_debug_dict(self.virtual_measure_pose),
            'follow_anchor_pose': self._pose_to_debug_dict(self.follow_anchor_pose),
            'nav_recovery_target': self._pose_to_debug_dict(self.nav_recovery_target),
            'nav_recovery_failed_status': self.nav_recovery_failed_status,
            'nav_recovery_ready_time_sec': self._finite_float_or_none(self.nav_recovery_ready_time_sec),
            'nav_recovery_attempts': self.nav_recovery_attempts,
            'measurement_count': len(self.measurement_track.points),
            'memory_readings_count': len(self.memory_readings.readings),
        }

        msg = String()
        msg.data = json.dumps(debug_state, ensure_ascii=False, allow_nan=False)
        self.debug_publisher.publish(msg)

    @staticmethod
    def _finite_float_or_none(value):
        return value if math.isfinite(value) else None

    @staticmethod
    def _degrees_or_none(value):
        return math.degrees(value) if math.isfinite(value) else None

    @staticmethod
    def _pose_to_debug_dict(pose):
        if pose is None:
            return None
        return {
            'x': MissionNode._finite_float_or_none(pose.x),
            'y': MissionNode._finite_float_or_none(pose.y),
            'z': MissionNode._finite_float_or_none(pose.z),
            'yaw': MissionNode._finite_float_or_none(pose.yaw),
            'yaw_deg': MissionNode._degrees_or_none(pose.yaw),
        }

    @staticmethod
    def _reading_to_debug_dict(reading):
        if reading is None:
            return None
        magnetic_field = reading.magnetic_field
        return {
            'signal_strength': MissionNode._finite_float_or_none(reading.signal_strength),
            'signal_strength_percent': MissionNode._finite_float_or_none(reading.signal_strength_percent),
            'depth': MissionNode._finite_float_or_none(reading.depth),
            'current': MissionNode._finite_float_or_none(reading.current),
            'pipeline_heading_degrees': MissionNode._finite_float_or_none(reading.pipeline_heading_degrees),
            'left_arrow': reading.left_arrow,
            'right_arrow': reading.right_arrow,
            'stamp_sec': MissionNode._finite_float_or_none(reading.stamp_sec),
            'frame_id': reading.frame_id,
            'magnetic_field': MissionNode._vector_to_debug_dict(magnetic_field),
        }

    @staticmethod
    def _vector_to_debug_dict(vector):
        if vector is None:
            return None
        return {
            'x': MissionNode._finite_float_or_none(vector.x),
            'y': MissionNode._finite_float_or_none(vector.y),
            'z': MissionNode._finite_float_or_none(vector.z),
        }

    def _advance_state_machine(self):
        if self.done or self.robot.pose is None or self.robot.reading is None:
            return

        completed_goal = self.robot.consume_completed_goal()
        if completed_goal is not None:
            _, goal_succeeded, goal_status = completed_goal
            if not goal_succeeded:
                if goal_status == GoalStatus.STATUS_ABORTED:
                    self._start_nav_recovery(goal_status)
                    return
                self._complete(
                    MissionState.FAILED,
                    'Navigation goal failed with status=%s.' % goal_status,
                )
                return
            self._require_fresh_reading()
        elif completed_goal is None:
            completed_goal = self.robot.pose, True, STATUS_SUCCEEDED # 注意后面这里不能要
        elif self.robot.has_pending_goal():
            return

        for _ in range(8):
            if self.done or self.robot.has_pending_goal():
                return
            if not self._tick_state_without_active_goal(completed_goal):
                return
            completed_goal = None

    def _require_fresh_reading(self):
        self.need_fresh_reading = True
        self.fresh_reading_after_stamp = self.robot.reading.stamp_sec

    def _has_fresh_reading(self):
        return self.robot.reading.stamp_sec > self.fresh_reading_after_stamp

    def _clear_fresh_reading_requirement(self):
        self.need_fresh_reading = False
        self.fresh_reading_after_stamp = 0.0

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _select_nav_recovery_target(self):
        if self.measurement_track.points:
            return self.measurement_track.points[-1].pose
        return self.first_confirmed_measurement_pose

    def _reset_transient_navigation_context(self):
        self._clear_center_bracket()
        self._clear_virtual_measurement()
        self.rotation_count = 0
        self.max_yaw = 0
        self.max_signal = float('-inf')
        self.min_yaw = 0
        self.min_signal = float('inf')
        self.last_search_target_yaw = 0.0
        self.search_base_yaw = 0.0
        self.standard = None
        self.repeat_count = 0
        self.conner_check_lock = False
        self.confirmed_pipeline_yaw = None
        self.memory_readings = MemoryReadings(memory_readings_capacity=self.memory_readings_capacity)

    def _start_nav_recovery(self, failed_status):
        recovery_target = self._select_nav_recovery_target()
        if recovery_target is None:
            self._complete(
                MissionState.FAILED,
                'Navigation goal aborted with status=%s, and no confirmed recovery point is available.' % failed_status,
            )
            return

        self.nav_recovery_attempts += 1
        if self.nav_recovery_attempts > self.max_nav_recovery_attempts:
            self._complete(
                MissionState.FAILED,
                'Navigation recovery exceeded max attempts=%d after status=%s.' % (
                    self.max_nav_recovery_attempts,
                    failed_status,
                ),
            )
            return

        self._reset_transient_navigation_context()
        self.nav_recovery_target = recovery_target
        self.nav_recovery_failed_status = failed_status
        self.nav_recovery_ready_time_sec = self._now_sec() + self.nav_recovery_wait_seconds
        self.wait_reading_after_stamp = self.robot.reading.stamp_sec
        self._require_fresh_reading()
        self.phase = "nav_recovery_wait_fresh_reading"
        self._transition(MissionState.REACQUIRE, 'navigation aborted; recover to confirmed measurement')

    def _tick_state_without_active_goal(self, completed_goal=None):
        # Phase handler returns None when no phase is active, True when the
        # state machine may keep ticking, and False when it must wait.
        phase_result = self._tick_phase_without_active_goal(completed_goal)
        if phase_result is not None:
            return phase_result

        if completed_goal is not None:
            self._record_log('arrived at active goal')

        if self.need_fresh_reading:
            if not self._has_fresh_reading():
                return False
            self._clear_fresh_reading_requirement()

        if self.step_count >= self.max_steps:
            self._complete(MissionState.FAILED, 'Mission reached max_steps=%d.' % self.max_steps)
            return False

        return self._tick_mission_state_without_active_goal()

    def _tick_phase_without_active_goal(self, completed_goal=None):
        goal_pose = completed_goal[0] if completed_goal is not None else None

        if self.phase == "nav_recovery_wait_fresh_reading":
            if self.nav_recovery_target is None:
                self._complete(MissionState.FAILED, 'Navigation recovery has no target.')
                return False
            if self._now_sec() < self.nav_recovery_ready_time_sec:
                return False
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return False
            self._clear_fresh_reading_requirement()
            target = self.nav_recovery_target
            self.phase = "nav_recovery_return"
            self._issue_move_to(target.x, target.y, target.yaw, 'navigation recovery')
            return False

        if self.phase == "nav_recovery_return":
            if completed_goal is None:
                return False
            self.nav_recovery_target = None
            self.nav_recovery_failed_status = None
            self.nav_recovery_ready_time_sec = 0.0
            self.nav_recovery_attempts = 0
            self.phase = "normal"
            self._transition(MissionState.REACQUIRE, 'navigation recovery complete')
            return True

        if self.phase == "rotation_trial to search" or self.phase == "rotation_trial to reacquire":
            if goal_pose is None:
                return False
            self.last_search_target_yaw = goal_pose.yaw
            self.wait_reading_after_stamp = self.robot.reading.stamp_sec
            self.phase = "rotation_trial_wait_reading to search" if self.phase == "rotation_trial to search" else "rotation_trial_wait_reading to reacquire"
            return False
            
        if self.phase == "rotation_trial_wait_reading to search" or self.phase == "rotation_trial_wait_reading to reacquire":
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return False
            else:
                self._clear_fresh_reading_requirement()
                # self.get_logger().info('turnthephase')
                self.phase = "rotation_trial to search" if self.phase == "rotation_trial_wait_reading to search" else "rotation_trial to reacquire"
                self.rotation_count += 1
                receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
                if self.robot.reading.left_arrow or self.robot.reading.right_arrow:
                    self.phase = "normal"
                    self.rotation_count = 0
                    self.max_yaw = 0
                    self.max_signal = float('-inf')
                    self.min_yaw = 0
                    self.min_signal = float('inf')
                    self._transition(MissionState.CENTER_ON_LINE, 'detected magnetic signal during rotation trial')
                    return True
                if self.max_signal < self.robot.reading.signal_strength_percent:
                    self.max_yaw = self.last_search_target_yaw # 无噪声
                    self.max_signal = self.robot.reading.signal_strength_percent
                if self.min_signal > self.robot.reading.signal_strength_percent:
                    self.min_yaw = self.last_search_target_yaw
                    self.min_signal = self.robot.reading.signal_strength_percent
                if self.rotation_count < 12:
                    next_yaw = (self.search_base_yaw + self.rotation_count * math.radians(30)) % (2 * math.pi)
                    if self.rotate_around_robot and self.phase == "rotation_trial to search":
                        self.robot.robot_move_to(self.robot.pose.x, self.robot.pose.y, next_yaw)    
                    else:
                        self._issue_move_to(receiver_pose.x, receiver_pose.y, next_yaw)
                    return False
                else:
                    next_step = self.search_probe_distance if self.phase == "rotation_trial to search" else self.reacquire_probe_offset
                    if self.confirmed_pipeline_yaw is not None:
                        included_angle = (self.min_yaw - self.confirmed_pipeline_yaw) % (2 * math.pi)
                    if self.confirmed_pipeline_yaw is not None and (math.radians(90) < included_angle < math.radians(270)):
                        next_yaw = (self.min_yaw + math.pi) % (2 * math.pi)
                    else:
                        next_yaw = self.min_yaw
                    dx = math.cos(next_yaw) * next_step 
                    dy = math.sin(next_yaw) * next_step
                    # 如果没有确定的方向，就要先通过同方向不同位置的max_signal大小判断。这里是将移动与旋转两步合为一步
                    if self.confirmed_pipeline_yaw is None:
                        next_yaw = self.max_yaw 
                    if self.rotate_around_robot and self.phase == "rotation_trial to search":
                        target_x = self.robot.pose.x + dx
                        target_y = self.robot.pose.y + dy
                        self.robot.robot_move_to(target_x, target_y, next_yaw)
                    else:
                        target_x = receiver_pose.x + dx
                        target_y = receiver_pose.y + dy
                        self._issue_move_to(target_x, target_y, next_yaw)
                    if self.confirmed_pipeline_yaw is None:# 需要确认管线方向
                        if self.phase == "rotation_trial to search":
                            self.phase = "confirm_pipeline_yaw_rotate_around_robot" 
                        else:
                            self.phase = "confirm_pipeline_yaw_rotate_around_receiver" 
                        self.get_logger().info("turn into confirm phase")
                    else:
                        self.rotation_count = 0
                        self.max_yaw = 0
                        self.max_signal = float('-inf')
                        self.min_yaw = 0
                        self.min_signal = float('inf')
                        self.phase = "normal"
                    return False
                return False

        if self.phase == "confirm_pipeline_yaw_rotate_around_robot" or self.phase == "confirm_pipeline_yaw_rotate_around_receiver":
            if completed_goal is None:
                return False
            self.wait_reading_after_stamp = self.robot.reading.stamp_sec
            self.phase = "confirm_pipeline_yaw_wait_reading"
            return False

        if self.phase == "confirm_pipeline_yaw_wait_reading":
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return False
            else:
                self._clear_fresh_reading_requirement()
                if self.max_signal < self.robot.reading.signal_strength_percent: # 错误方向
                    self.confirmed_pipeline_yaw = (self.max_yaw + math.pi) % (2 * math.pi)
                else:
                    self.confirmed_pipeline_yaw = self.max_yaw
            self.get_logger().info("---->We find confirmed pipeline yaw: %.2f degrees" % math.degrees(self.confirmed_pipeline_yaw))
            self.rotation_count = 0
            self.max_yaw = 0
            self.max_signal = float('-inf')
            self.min_yaw = 0
            self.min_signal = float('inf')
            self.phase = "normal"
            return True
                    


        # wait for accurate reading after move
        if self.phase == "wait_accurate_reading":
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return False
            self._clear_fresh_reading_requirement()
            reading = self.robot.reading
            if (not reading.left_arrow) and (not reading.right_arrow):
                self.phase = "normal"
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self._transition(MissionState.REACQUIRE, 'measurement rejected: off arrows')
                return True
            if (not reading.left_arrow) or (not reading.right_arrow):
                self.phase = "normal"
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'measurement rejected after fresh reading: off-center')
                return True
            self.phase = "normal"
            self._clear_center_bracket()
            self._clear_virtual_measurement()
            # 寻找转点
            receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
            self._record_corner_if_needed(receiver_pose)
            self._record_measurement_point('measurement point recorded')
            self._transition(MissionState.FOLLOW_LINE, 'measurement complete')
            return True

        # 一步步退回第一个测量点
        if self.phase == "return to first confirmed measurement point":
            last_point = self.measurement_track_copy.get_last_point()
            if last_point is None:
                self.phase = "normal"
                self._transition(MissionState.REACQUIRE, 'return complete')
                return True
            receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
            last_point_x = last_point.pose.x
            last_point_y = last_point.pose.y
            while self.check_out_workspace_boundary(last_point_x, last_point_y, receiver_pose) and last_point is not None:
                last_point = self.measurement_track_copy.get_last_point()
                self.get_logger().info("-->left:%d" % len(self.measurement_track_copy.points))
                if last_point is not None:
                    last_point_x = last_point.pose.x
                    last_point_y = last_point.pose.y
            if last_point is None:
                self.phase = "normal"
                self._transition(MissionState.REACQUIRE, 'return complete')
                return True
            oppo_last_yaw = (last_point.pose.yaw + math.pi) % (2 * math.pi)
            self._issue_move_to(last_point.pose.x, last_point.pose.y, oppo_last_yaw)
            return False

        return None

    def _tick_mission_state_without_active_goal(self):
        reading = self.robot.reading
        signal = reading.signal_strength_percent
        

        if self.state == MissionState.SEARCH_PEAK:
            if reading.left_arrow or reading.right_arrow: # notice: should be converted to l-r arrows
                self._transition(MissionState.CENTER_ON_LINE, 'detected magnetic signal')
                return True
            self._issue_searching_move('search peak')
            return False 

        if self.state == MissionState.CENTER_ON_LINE:
            if reading.left_arrow and reading.right_arrow:
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self.line_confirmed = True # notice: whether to trust the arrows
                if self.first_confirmed_measurement_pose is None:
                    receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
                    self.first_confirmed_measurement_pose = receiver_pose
                self.follow_moves_since_center = 0 # notice: seems not necessary
                self._transition(MissionState.MEASURE_ON_LINE, 'centered on magnetic line')
                return True
            elif (not reading.left_arrow) and (not reading.right_arrow):
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self._transition(MissionState.REACQUIRE, 'lost magnetic line')
                return True

            single_arrow = self._single_arrow_side(reading)
            receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
            if (
                single_arrow is not None
                and self.center_last_single_arrow is not None
                and single_arrow != self.center_last_single_arrow
                and self.center_last_single_pose is not None
            ):
                virtual_pose = self._midpoint_pose(self.center_last_single_pose, receiver_pose)
                virtual_reading = self._centered_reading_from(reading)
                self._clear_center_bracket()
                self.virtual_measure_pose = virtual_pose
                self.virtual_measure_reading = virtual_reading
                self.follow_anchor_pose = virtual_pose
                self.line_confirmed = True
                if self.first_confirmed_measurement_pose is None:
                    self.first_confirmed_measurement_pose = virtual_pose
                self.follow_moves_since_center = 0
                self._transition(MissionState.MEASURE_ON_LINE, 'centered on magnetic line by bracketed arrows')
                return True

            # repeat for to many times -> endpoint
            robotposition = Point3D(receiver_pose.x, receiver_pose.y, 0)
            if self.endpoints_judge(point=robotposition):
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                if self.line_confirmed and self.start_second_search:
                    self._complete(MissionState.COMPLETE, 'endpoint reached')
                elif self.line_confirmed and (not self.start_second_search):
                    self.start_second_search = True
                    self.start_second_search_turning = True
                    # self._issue_move_to(self.first_confirmed_measurement_pose.x, self.first_confirmed_measurement_pose.y, self.first_confirmed_measurement_pose.yaw)
                    self._transition(MissionState.REACQUIRE, 'return to first confirmed measurement point')
                    self.get_logger().info('return to first confirmed measurement point, and start the second search')
                    self.phase = 'return to first confirmed measurement point'
                    self.measurement_track_copy = copy.deepcopy(self.measurement_track)
                    self.reverse_number = len(self.measurement_track.points)
                else:
                    self._complete(MissionState.FAILED, 'repetation for unknown reasons')
                return False

            self.center_last_single_arrow = single_arrow
            self.center_last_single_pose = receiver_pose
            self.center_last_single_reading = reading
            self._issue_lateral_move(self.centering_step, 'center on line')
            return False

        if self.state == MissionState.MEASURE_ON_LINE:
            if self.virtual_measure_pose is not None and self.virtual_measure_reading is not None:
                self.confirmed_pipeline_yaw = None
                if self.conner_check_lock:
                    self.conner_check_lock = False
                self._record_corner_if_needed(self.virtual_measure_pose)
                self._record_measurement_point(
                    'virtual measurement point recorded from bracketed arrows',
                    pose_override=self.virtual_measure_pose,
                    reading_override=self.virtual_measure_reading,
                )
                self.virtual_measure_pose = None
                self.virtual_measure_reading = None
                self._transition(MissionState.FOLLOW_LINE, 'measurement complete')
                return True

            if (not reading.left_arrow) and (not reading.right_arrow):
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self._transition(MissionState.REACQUIRE, 'measurement rejected: off arrows')
                return True

            if (not reading.left_arrow) or (not reading.right_arrow):
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'measurement rejected: off-center')
                return True

            # 后面的REACQUIRE需要重新确认
            self.confirmed_pipeline_yaw = None

            if self.conner_check_lock:
                self.conner_check_lock = False # firstly ensure the machine on the line, then unlock
            
            self.wait_reading_after_stamp = self.robot.reading.stamp_sec
            self.phase = "wait_accurate_reading"
            return False
            
            # notice: The reading of results may be slower than the reading action
            # self._record_log('measurement on line')
            # self._record_measurement_point('measurement point recorded')
            # self._transition(MissionState.FOLLOW_LINE, 'measurement complete')
            # return True

        if self.state == MissionState.FOLLOW_LINE:
            if self.follow_anchor_pose is not None:
                self._transition(MissionState.MEASURE_ON_LINE, 'measure after follow move')
                self._issue_forward_move('follow line')
                return False

            if (not reading.left_arrow) and (not reading.right_arrow): 
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self._transition(MissionState.REACQUIRE, 'magnetic signal off arrows')
                return True
            elif (not reading.left_arrow) or (not reading.right_arrow):
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'magnetic signal off-center')
                return True
            # if self.follow_moves_since_center >= self.orientation_check_interval:
            #     self.follow_moves_since_center = 0
            #     self._transition(MissionState.CENTER_ON_LINE, 'periodic centering check')
            #     return True
            self._transition(MissionState.MEASURE_ON_LINE, 'measure after follow move')
            self._issue_forward_move('follow line')
            # self.follow_moves_since_center += 1
            return False

        # Differences from the searching state: conner check
        # notice: some conner will still be missed, especially the case when the robot dont need to reacquire around the conner
        if self.state == MissionState.REACQUIRE:
            if reading.left_arrow and reading.right_arrow:
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self._transition(MissionState.MEASURE_ON_LINE, 'reacquired magnetic signal')
                return True
            if reading.left_arrow or reading.right_arrow:
                self._clear_center_bracket()
                self._clear_virtual_measurement()
                self._transition(MissionState.CENTER_ON_LINE, 'reacquired magnetic signal')
                return True
            
            self.get_logger().info('--->get reacquire')

            # back to the conner
            if len(self.memory_readings.readings) > 0 and (not self.conner_check_lock):
                oldest_reading = self.memory_readings.get_oldest_reading()
                # 下面current_orientation要注意REACQUIRE可能没有pipline_heading_degrees，导致current_orientation计算错误
                current_orientation = self.robot.pose.yaw % (2 * math.pi)
                oldest_orientation = (oldest_reading.pose.yaw + math.radians(oldest_reading.pipeline_heading_degrees)) % (2 * math.pi)
                included_angle = (current_orientation - oldest_orientation) % (2 * math.pi)
                if self.check_conner(included_angle):
                    toward = (self.robot.pose.yaw + math.pi) % (2 * math.pi)
                    dx = 3 * math.cos(toward) * self.reacquire_probe_offset # no
                    dy = 3 * math.sin(toward) * self.reacquire_probe_offset
                    self._issue_move_by(dx, dy, toward, 'reacquire line')
                    self.conner_check_lock = True
                    self.get_logger().info('we find a conner, and we will back to the conner')
                    return False
            

            self._issue_searching_move('reacquire line')
            return False

        return False

    def _single_arrow_side(self, reading):
        if reading.left_arrow and not reading.right_arrow:
            return 'left'
        if reading.right_arrow and not reading.left_arrow:
            return 'right'
        return None

    def _clear_center_bracket(self):
        self.center_last_single_arrow = None
        self.center_last_single_pose = None
        self.center_last_single_reading = None

    def _clear_virtual_measurement(self):
        self.virtual_measure_pose = None
        self.virtual_measure_reading = None
        self.follow_anchor_pose = None

    def _midpoint_pose(self, pose_a, pose_b):
        sin_sum = math.sin(pose_a.yaw) + math.sin(pose_b.yaw)
        cos_sum = math.cos(pose_a.yaw) + math.cos(pose_b.yaw)
        yaw = pose_b.yaw if math.hypot(sin_sum, cos_sum) <= 1e-9 else math.atan2(sin_sum, cos_sum)
        return RobotPose(
            x=(pose_a.x + pose_b.x) / 2.0,
            y=(pose_a.y + pose_b.y) / 2.0,
            z=(pose_a.z + pose_b.z) / 2.0,
            yaw=yaw % (2 * math.pi),
        )

    def _centered_reading_from(self, reading):
        return ReceiverReading(
            signal_strength=reading.signal_strength,
            depth=reading.depth,
            current=reading.current,
            pipeline_heading_degrees=reading.pipeline_heading_degrees,
            signal_strength_percent=reading.signal_strength_percent,
            left_arrow=True,
            right_arrow=True,
            stamp_sec=reading.stamp_sec,
            frame_id=reading.frame_id,
            magnetic_field=reading.magnetic_field,
        )

    def _issue_searching_move(self, reason): # notice: todo
        self._clear_center_bracket()
        self._clear_virtual_measurement()
        if self.robot.reading.left_arrow or self.robot.reading.right_arrow:
            return
        self.phase = "rotation_trial to search" if reason == "search peak" else "rotation_trial to reacquire"
        self.rotation_count = 1
        receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose) 
        if self.rotate_around_robot and reason == "search peak": # 这种情况下可以围绕着小狗旋转
            self.search_base_yaw = self.robot.pose.yaw
        else:
            self.search_base_yaw = receiver_pose.yaw
        self.max_yaw = self.search_base_yaw
        self.max_signal = self.robot.reading.signal_strength_percent
        self.min_yaw = self.search_base_yaw
        self.min_signal = self.robot.reading.signal_strength_percent
        next_yaw = (self.search_base_yaw + self.rotation_count * math.radians(30)) % (2 * math.pi)
        # self.get_logger().info('target: %.3f, now: %.3f' % (math.degrees(next_yaw), math.degrees(self.robot.pose.yaw)))
        if self.rotate_around_robot and reason == "search peak":
            # self.get_logger().info(">-------------<")
            
            self.robot.robot_move_to(self.robot.pose.x, self.robot.pose.y, next_yaw)
        else:
            self._issue_move_to(receiver_pose.x, receiver_pose.y, next_yaw)
        return

    def _issue_forward_move(self, reason): # 系统性误差：机器人没有严格旋转到位就进行下一步
        anchor_pose = self.follow_anchor_pose
        base_yaw = anchor_pose.yaw if anchor_pose is not None else self.robot.pose.yaw
        heading_rad = math.radians(self.robot.reading.pipeline_heading_degrees) + base_yaw # notice: radian or degree
        heading_rad = heading_rad % (2 * math.pi)  
        dx = math.cos(heading_rad) * self.forward_step
        dy = math.sin(heading_rad) * self.forward_step
        # if self.start_second_search_turning:
        #     dx, dy, heading_rad = -dx, -dy, (heading_rad + math.pi) % (2 * math.pi)
        #     self.start_second_search_turning = False
        self.memory_readings.add_reading(RobotState(self.robot.pose, self.robot.reading))
        if anchor_pose is not None:
            self.follow_anchor_pose = None
            self._issue_move_to(anchor_pose.x + dx, anchor_pose.y + dy, heading_rad, reason)
        else:
            self._issue_move_by(dx, dy, heading_rad, reason)
        return

    def _issue_lateral_move(self, step_size, reason):
        reading = self.robot.reading
        if reading.left_arrow and reading.right_arrow:
            return # notice: 是否会有潜在的问题
        if not reading.left_arrow and not reading.right_arrow:
            return
        
        side = -1.0 if reading.left_arrow else 1.0
        receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
        yaw = receiver_pose.yaw
        toward = (yaw + math.radians(reading.pipeline_heading_degrees) + (side * math.pi) / 2) % (2 * math.pi)
        dx = math.cos(toward) * step_size
        dy = math.sin(toward) * step_size
        # self.get_logger().info("yaw = %.3f, toward = %.3f \n" % (math.degrees(yaw), math.degrees(toward)))

        self._issue_move_by(dx, dy, yaw, reason)
        return

    def _issue_move_by(self, dx, dy, yaw, reason):
        pose = self.robot.pose
        receiver_pose = self.robot.robot_tf_to_receiver(pose)
        target_x = receiver_pose.x + dx
        target_y = receiver_pose.y + dy
        if self.check_out_workspace_boundary(target_x, target_y, receiver_pose):
            return

        self._move_to_and_memorise(target_x, target_y, yaw)
        self.step_count += 1
        self.get_logger().info(
            'Step %d state=%s target: x=%.2f y=%.2f yaw=%.2f reason=%s signal=%.3f'
            % (
                self.step_count,
                self.state.value,
                target_x,
                target_y,
                yaw,
                reason,
                self.robot.reading.signal_strength_percent,
            )
        )

    def _issue_move_to(self, target_x, target_y, yaw, reason = ""):
        pose = self.robot.pose
        receiver_pose = self.robot.robot_tf_to_receiver(pose)
        # self.get_logger().info(">-------------<")
        if self.check_out_workspace_boundary(target_x, target_y, receiver_pose):
            return

        # self.get_logger().info("<------------->")
        self._move_to_and_memorise(target_x, target_y, yaw)
        self.step_count += 1
        self.get_logger().info(
            'Step %d state=%s target: x=%.2f y=%.2f yaw=%.2f reason=%s signal=%.3f'
            % (
                self.step_count,
                self.state.value,
                target_x,
                target_y,
                yaw,
                reason,
                self.robot.reading.signal_strength_percent,
            )
        )

    def check_conner(self, included_angle):
        return (math.radians(30) < included_angle < math.radians(150)) or (math.radians(210) < included_angle < math.radians(330))

    def check_out_workspace_boundary(self, target_x, target_y, receiver_pose):
        if not (self.workspace_min_x <= target_x <= self.workspace_max_x and self.workspace_min_y <= target_y <= self.workspace_max_y):
            self._clear_center_bracket()
            self._clear_virtual_measurement()
            if self.phase == "return to first confirmed measurement point":
                return True
            elif self.line_confirmed and self.start_second_search:
                self._complete(MissionState.COMPLETE, 'workspace boundary reached')
            elif self.line_confirmed and (not self.start_second_search):
                self.start_second_search = True
                self.start_second_search_turning = True # only turn once
                # self._issue_move_to(self.first_confirmed_measurement_pose.x, self.first_confirmed_measurement_pose.y, self.first_confirmed_measurement_pose.yaw)
                self._transition(MissionState.REACQUIRE, 'return to first confirmed measurement point')
                self.get_logger().info('return to first confirmed measurement point, and start the second search')
                self.phase = 'return to first confirmed measurement point'
                self.measurement_track_copy = copy.deepcopy(self.measurement_track)
                self.reverse_number = len(self.measurement_track.points)
            else:
                self._complete(MissionState.FAILED, 'workspace boundary reached, but line is not confirmed')
            return True # 不在边界中
        # if math.hypot(target_x - receiver_pose.x, target_y - receiver_pose.y) <= 1e-9:
        #     self.get_logger().info("---->")
        #     self._transition(MissionState.REACQUIRE, 'no available motion in workspace')
        #     return True # 非法移动检查，并进来了
        return False

    def _move_to_and_memorise(self, x, y, yaw):
        # self.get_logger().info("receiver move to (%.2f, %.2f), yaw = %.2f\nnow in (%.2f, %.2f), yaw = %.2f" % (x, y, yaw, receiver_pose.x, receiver_pose.y, receiver_pose.yaw))
        self.robot.receiver_move_to(x, y, yaw)

    def _is_close(self, point1: Point3D, point2: Point3D) -> bool:
        return math.dist((point1.x, point1.y), (point2.x, point2.y)) <= self.close_threshold

    def endpoints_judge(self, point: Point3D) -> bool:
        if self.standard is None:
            self.standard = point
        if self._is_close(self.standard, point):
            self.repeat_count += 1
        else:
            self.repeat_count = 0
            self.standard = point
        return self.repeat_count >= self.repeat_count_threshold


    def _transition(self, next_state, reason):
        if self.state == next_state:
            return
        self.get_logger().info('%s -> %s: %s' % (self.state.value, next_state.value, reason))
        self.state = next_state

    def _record_log(self, note):
        if self.robot.pose is None or self.robot.reading is None:
            return
        self.log.append(
            MissionLogEntry(
                step=self.step_count,
                state=self.state,
                pose=self.robot.pose,
                reading=self.robot.reading,
                note=note,
            )
        )
        self.get_logger().info(
            'Log state=%s pose=(%.2f, %.2f) signal=%.3f note=%s'
            % (
                self.state.value,
                self.robot.pose.x,
                self.robot.pose.y,
                self.robot.reading.signal_strength_percent,
                note,
            )
        )
    
    def _record_corner_if_needed(self, current_pose):
        if current_pose is None or len(self.measurement_track.points) == 0:
            return
        last_measurement = self.measurement_track.points[-1]
        last_yaw = last_measurement.pose.yaw
        current_yaw = current_pose.yaw
        included_angle = (current_yaw - last_yaw) % (2 * math.pi)
        if self.check_conner(included_angle):
            self.get_logger().info("---->We find a conner, and we will record the conner")
            self.conners_number.append(len(self.measurement_track.points))
            self.get_logger().info('1.conner number = %s, conner point = %s' % (self.conners_number, [self.measurement_track.points[i - 1].pose for i in self.conners_number]))

    def _record_measurement_point(self, note, pose_override=None, reading_override=None):
        reading = reading_override if reading_override is not None else self.robot.reading
        if self.robot.pose is None or reading is None:
            return
        if not(reading.left_arrow and reading.right_arrow):
            return
        receiver_pose = pose_override if pose_override is not None else self.robot.robot_tf_to_receiver(self.robot.pose)
        self.measurement_track.append(
            step = self.step_count,
            state = self.state,
            pose = receiver_pose,
            reading = reading,
            note = note
        )


    def _complete(self, final_state, reason):
        self.done = True
        self.state = final_state
        self._record_log(reason)
        self.get_logger().info('Mission finished with state=%s: %s' % (final_state.value, reason))
        
        record_number = len(self.measurement_track.points)
        if self.reverse_number is not None and 0 < self.reverse_number < record_number:
            splice_points = self.measurement_track.points[(self.reverse_number - 1) :  : -1] + self.measurement_track.points[self.reverse_number : record_number]
        else:
            splice_points = self.measurement_track.points[:]

        self.get_logger().info('reverse_number = %d' % self.reverse_number)
        records = [i.pose for i in self.measurement_track.points]
        self.get_logger().info('measurement points = %s' % records)
        self.get_logger().info('------->splice points = %s' % [i.pose for i in splice_points])
        self.get_logger().info('2.conner number = %s, conner point = %s' % (self.conners_number, [self.measurement_track.points[i - 1].pose for i in self.conners_number]))
        for i in [self.reverse_number - 1, self.reverse_number, self.reverse_number + 1]:
            if i in self.conners_number:
                self.conners_number.remove(i)
        splice_conners_number = []
        for i in self.conners_number:
            if i < self.reverse_number:
                splice_conners_number.append(self.reverse_number - 2 - i)
            else:
                splice_conners_number.append(i)

        self.get_logger().info('3.conner number = %s, conner point = %s' % (self.conners_number, [self.measurement_track.points[i - 1].pose for i in self.conners_number]))
        self.get_logger().info('------->splice conner number = %s, splice conner point = %s' % (splice_conners_number, [splice_points[i].pose for i in splice_conners_number]))

        splice_conners_number.append(0)
        splice_conners_number.append(len(splice_points) - 1)
        splice_conners_number.sort()
        split_points = []
        for i in range(len(splice_conners_number)):
            if i == 0:
                continue
            else:
                former = splice_conners_number[i - 1] + 2  # conner附近的两个点认为不可信
                latter = splice_conners_number[i] - 1
                split_points.append(splice_points[former : latter])
        # self.get_logger().info('split points = %s' % split_points)
        # for i in split_points:
        #     self.get_logger().info('split points = %s' % type(i))

        for i in split_points:
            for j in i:
                self.get_logger().info('split points = (%.3f, %.3f)' % (j.pose.x, j.pose.y))
            self.get_logger().info('--------------------')

        split_points_coordinate_x = [np.fromiter((j.pose.x for j in i), dtype=float) for i in split_points]
        split_points_coordinate_y = [np.fromiter((j.pose.y for j in i), dtype=float) for i in split_points]
        split_points_coordinate_k = []
        split_points_coordinate_b = []
        for i in range(len(split_points)):
            if len(split_points_coordinate_x[i]) < 2:
                split_points_coordinate_k.append(None)
                split_points_coordinate_b.append(None)
            else:
                k, b = np.polyfit(split_points_coordinate_x[i], split_points_coordinate_y[i], 1) # 后面可以修改，不用kb
                split_points_coordinate_k.append(k)
                split_points_coordinate_b.append(b)
                self.get_logger().info('the line is y = %.3f * x + %.3f' % (k, b))
        

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
