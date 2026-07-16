import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from mission_logic_msgs.msg import SensorMsg
from std_msgs.msg import Float32
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
        goal_pose_publisher,
        speed_publisher,
        goal_frame='map',
        arrival_tolerance_distance = 0.2,
        arrival_tolerance_yaw_rad = math.radians(10.0),
        speed = 1.0,
        receiver_robot_dx = 1.0,
        receiver_robot_dy = 1.0,
        receiver_robot_same_direction = True
    ):
        self._node = node
        self._goal_pose_publisher = goal_pose_publisher
        self._speed_publisher = speed_publisher
        self._goal_frame = goal_frame
        self._arrival_tolerance_distance = arrival_tolerance_distance
        self._arrival_tolerance_yaw_rad = arrival_tolerance_yaw_rad
        self._speed = speed
        self.receiver_robot_dx = receiver_robot_dx
        self.receiver_robot_dy = receiver_robot_dy
        self.receiver_robot_same_direction = receiver_robot_same_direction

        self.pose: Optional[RobotPose] = None
        self.reading: Optional[ReceiverReading] = None
        self.active_goal: Optional[RobotPose] = None

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
        self._publish_speed()
        self.publish_active_goal()
        return MoveResult(target=self.active_goal, reading=self.reading)

    def robot_tf_to_receiver(self, pose: RobotPose):
        if self.pose is None:
            return None
        
        receiver_yaw = pose.yaw if self.receiver_robot_same_direction else (pose.yaw + math.pi) % (2 * math.pi)
        rr_distance = math.hypot(self.receiver_robot_dx, self.receiver_robot_dy)
        rr_yaw = math.atan2(self.receiver_robot_dy, self.receiver_robot_dx)
        receiver_x = pose.x + rr_distance * math.cos(receiver_yaw + rr_yaw)
        receiver_y = pose.y + rr_distance * math.sin(receiver_yaw + rr_yaw)
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

    def publish_active_goal(self):
        if self.active_goal is None:
            return

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

        self._goal_pose_publisher.publish(goal_pose_msg)
    
    def has_arrived_helpful_func(self, pose: RobotPose, goal: RobotPose): # 为了其他地方的调用拆出来的
        dx = pose.x - goal.x
        dy = pose.y - goal.y
        dyaw = pose.yaw - goal.yaw
        yaw_error = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
        return (
            math.hypot(dx, dy) <= self._arrival_tolerance_distance
            and yaw_error <= self._arrival_tolerance_yaw_rad
        )

    def has_arrived(self):
        if self.pose is None or self.active_goal is None:
            return False
        active_goal_pose = RobotPose(self.active_goal.x, self.active_goal.y, self.pose.z, self.active_goal.yaw)
        return self.has_arrived_helpful_func(self.pose, active_goal_pose)

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
        self.declare_parameter('arrival_tolerance_distance', 0.2)
        self.declare_parameter('arrival_tolerance_yaw_degree', 10.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_republish_period', 1.0)
        self.declare_parameter('max_steps', 100)
        self.declare_parameter('close_threshold', 1.0)
        self.declare_parameter('repeat_count_threshold', 4)
        self.declare_parameter('memory_readings_capacity', 5)
        self.declare_parameter('receiver_robot_dx', 1.0)
        self.declare_parameter('receiver_robot_dy', 1.0) # 注意config里面有两个这个需要设置！
        self.declare_parameter('receiver_robot_same_direction', True)

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
        self.goal_republish_period = self.get_parameter('goal_republish_period').value
        self.receiver_robot_dx = self.get_parameter('receiver_robot_dx').value
        self.receiver_robot_dy = self.get_parameter('receiver_robot_dy').value
        self.receiver_robot_same_direction = self.get_parameter('receiver_robot_same_direction').value
        # notice: follow_line step should > reacquire_step, for the correctness on conner

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.speed_publisher = self.create_publisher(Float32, '/speed', 10)

        self.robot = Robot(
            node=self,
            goal_pose_publisher=self.goal_pose_publisher,
            speed_publisher=self.speed_publisher,
            goal_frame=self.get_parameter('goal_frame').value,
            arrival_tolerance_distance=self.get_parameter('arrival_tolerance_distance').value,
            arrival_tolerance_yaw_rad=math.radians(
                self.get_parameter('arrival_tolerance_yaw_degree').value
                ),
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
        self.wait_reading_after_stamp = 0.0
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
        self.goal_timer = self.create_timer(
            self.goal_republish_period,
            self.robot.publish_active_goal,
        )
        self.control_timer = self.create_timer(0.2, self._advance_state_machine)
        self.measurement_track = LineMeasurementTrack()

    def state_estimation_callback(self, odometry_msg):
        self.robot.update_pose(odometry_msg)
        self._advance_state_machine()

    def magnetic_field_callback(self, magnetic_field_msg):
        self.robot.update_reading(magnetic_field_msg)
        self._advance_state_machine()

    def _advance_state_machine(self):
        if self.done or self.robot.pose is None or self.robot.reading is None:
            return
        

        # rotation_trial module
        if self.phase == "rotation_trial to search" or self.phase == "rotation_trial to reacquire":
            if self.robot.active_goal is not None and not self.robot.has_arrived():
                return
            if self.robot.has_arrived():
                self.last_search_target_yaw = self.robot.active_goal.yaw
                self.robot.active_goal = None
                self.wait_reading_after_stamp = self.robot.reading.stamp_sec
                self.phase = "rotation_trial_wait_reading to search" if self.phase == "rotation_trial to search" else "rotation_trial_wait_reading to reacquire"
                return

        if self.phase == "rotation_trial_wait_reading to search" or self.phase == "rotation_trial_wait_reading to reacquire":
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return
            else:
                # self.get_logger().info(
                #     "num = %d, signal = %.3f, yaw = %.2f, magnetic_field_y = %.3f \n" 
                #     % (self.rotation_count, self.robot.reading.signal_strength, math.degrees(self.last_search_target_yaw), self.robot.reading.magnetic_field.y))
                self.phase = "rotation_trial to search" if self.phase == "rotation_trial_wait_reading to search" else "rotation_trial to reacquire"
                self.rotation_count += 1
                if self.robot.reading.left_arrow or self.robot.reading.right_arrow:
                    self.phase = "normal"
                    self.rotation_count = 0
                    self.max_yaw = 0
                    self.max_signal = float('-inf')
                    self._transition(MissionState.CENTER_ON_LINE, 'detected magnetic signal during rotation trial')
                    return
                if self.max_signal < self.robot.reading.signal_strength:
                    self.max_yaw = self.last_search_target_yaw # 无噪声
                    self.max_signal = self.robot.reading.signal_strength
                if self.rotation_count < 12:
                    next_yaw = (self.search_base_yaw + self.rotation_count * math.radians(30)) % (2 * math.pi)
                    receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
                    if self.rotate_around_robot and self.phase == "rotation_trial to search":
                        self.robot.robot_move_to(self.robot.pose.x, self.robot.pose.y, next_yaw)    
                    else:
                        self.robot.receiver_move_to(receiver_pose.x, receiver_pose.y, next_yaw)
                    return
                else:
                    next_step = self.search_probe_distance if self.phase == "rotation_trial to search" else self.reacquire_probe_offset
                    dx = math.sin(self.max_yaw) * next_step
                    dy = -math.cos(self.max_yaw) * next_step
                    next_yaw = (self.max_yaw - math.pi / 2) % (2 * math.pi)
                    if self.phase == "rotation_trial_to_reacquire":
                        self._issue_move_by(dx, dy, next_yaw, 'reacquire')
                    else:
                        self._issue_move_by(dx, dy, next_yaw, 'search')
                    self.rotation_count = 0
                    self.max_yaw = 0
                    self.max_signal = float('-inf')
                    self.phase = "normal"
                    return
                return

        # wait for accurate reading after move
        if self.phase == "wait_accurate_reading":
            if self.robot.reading.stamp_sec <= self.wait_reading_after_stamp:
                return
            reading = self.robot.reading
            if (not reading.left_arrow) and (not reading.right_arrow):
                self.phase = "normal"
                self._transition(MissionState.REACQUIRE, 'measurement rejected: off arrows')
                return
            if (not reading.left_arrow) or (not reading.right_arrow):
                self.phase = "normal"
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'measurement rejected after fresh reading: off-center')
                return
            self.phase = "normal"
            self._record_log('measurement on line')
            self._record_measurement_point('measurement point recorded')
            self._transition(MissionState.FOLLOW_LINE, 'measurement complete')
            return

        if self.robot.active_goal is not None:
            if not self.robot.has_arrived():
                return
            self._record_log('arrived at active goal')
            self.robot.active_goal = None

        if self.step_count >= self.max_steps:
            self._complete(MissionState.FAILED, 'Mission reached max_steps=%d.' % self.max_steps)
            return

        for _ in range(8):
            if self.done or self.robot.active_goal is not None:
                return
            if not self._tick_state_without_active_goal(): # quickly skip state switching
                return

    def _tick_state_without_active_goal(self):
        reading = self.robot.reading
        signal = reading.signal_strength
        

        if self.state == MissionState.SEARCH_PEAK:
            if reading.left_arrow or reading.right_arrow: # notice: should be converted to l-r arrows
                self._transition(MissionState.CENTER_ON_LINE, 'detected magnetic signal')
                return True
            self._issue_searching_move('search peak')
            return False 

        if self.state == MissionState.CENTER_ON_LINE:
            if reading.left_arrow and reading.right_arrow: # notice: check
                self.line_confirmed = True # notice: whether to trust the arrows
                if self.first_confirmed_measurement_pose is None:
                    receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
                    self.first_confirmed_measurement_pose = receiver_pose
                self.follow_moves_since_center = 0 # notice: seems not necessary
                self._transition(MissionState.MEASURE_ON_LINE, 'centered on magnetic line')
                return True
            elif (not reading.left_arrow) and (not reading.right_arrow):
                self._transition(MissionState.REACQUIRE, 'lost magnetic line')
                return True
            
            # repeat for to many times -> endpoint
            receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
            robotposition = Point3D(receiver_pose.x, receiver_pose.y, 0)
            if self.endpoints_judge(point=robotposition):
                if self.line_confirmed and self.start_second_search:
                    self._complete(MissionState.COMPLETE, 'endpoint reached')
                elif self.line_confirmed and (not self.start_second_search):
                    self.start_second_search = True
                    self.start_second_search_turning = True
                    self.robot.receiver_move_to(self.first_confirmed_measurement_pose.x, self.first_confirmed_measurement_pose.y, self.first_confirmed_measurement_pose.yaw)
                    self._transition(MissionState.REACQUIRE, 'return to first confirmed measurement point')
                    self.get_logger().info('return to first confirmed measurement point, and start the second search')
                else:
                    self._complete(MissionState.FAILED, 'repetation for unknown reasons')
                return False
            
            self._issue_lateral_move(self.centering_step, 'center on line')
            return False

        if self.state == MissionState.MEASURE_ON_LINE:
            if (not reading.left_arrow) and (not reading.right_arrow):
                self._transition(MissionState.REACQUIRE, 'measurement rejected: off arrows')
                return True

            if (not reading.left_arrow) or (not reading.right_arrow):
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'measurement rejected: off-center')
                return True

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
            if (not reading.left_arrow) and (not reading.right_arrow): 
                self._transition(MissionState.REACQUIRE, 'magnetic signal off arrows')
                return True
            elif (not reading.left_arrow) or (not reading.right_arrow):
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'magnetic signal off-center')
                return True
            if self.follow_moves_since_center >= self.orientation_check_interval:
                self.follow_moves_since_center = 0
                self._transition(MissionState.CENTER_ON_LINE, 'periodic centering check')
                return True
            self._issue_forward_move('follow line')
            self.follow_moves_since_center += 1
            self._transition(MissionState.MEASURE_ON_LINE, 'measure after follow move')
            return False

        # Differences from the searching state: conner check
        # notice: some conner will still be missed, especially the case when the robot dont need to reacquire around the conner
        if self.state == MissionState.REACQUIRE:
            if reading.left_arrow and reading.right_arrow:
                self._transition(MissionState.MEASURE_ON_LINE, 'reacquired magnetic signal')
                return True
            if reading.left_arrow or reading.right_arrow:
                self._transition(MissionState.CENTER_ON_LINE, 'reacquired magnetic signal')
                return True
            
            # back to the conner
            if len(self.memory_readings.readings) > 0 and (not self.conner_check_lock):
                oldest_reading = self.memory_readings.get_oldest_reading()
                corrunt_orientation = (self.robot.pose.yaw + math.radians(self.robot.reading.pipeline_heading_degrees )) % (2 * math.pi)
                oldest_orientation = (oldest_reading.pose.yaw + math.radians(self.robot.reading.pipeline_heading_degrees )) % (2 * math.pi)
                angle_difference = (corrunt_orientation - oldest_orientation) % (2 * math.pi)
                if math.radians(45) < angle_difference < math.radians(315):
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

    def _issue_searching_move(self, reason): # notice: todo
        if self.robot.reading.left_arrow or self.robot.reading.right_arrow:
            return
        self.phase = "rotation_trial to search" if reason == "search peak" else "rotation_trial to reacquire"
        self.rotation_count = 0
        receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose) 
        if self.rotate_around_robot and reason == "search peak": # 这种情况下可以围绕着小狗旋转
            self.search_base_yaw = self.robot.pose.yaw
        else:
            self.search_base_yaw = receiver_pose.yaw
        self.max_yaw = self.search_base_yaw
        self.max_signal = float('-inf')
        next_yaw = (self.search_base_yaw + self.rotation_count * math.radians(30)) % (2 * math.pi)
        if self.rotate_around_robot and reason == "search peak":
            self.robot.robot_move_to(self.robot.pose.x, self.robot.pose.y, next_yaw)
        else:
            self._move_to_and_memorise(receiver_pose.x, receiver_pose.y, next_yaw)
        return

    def _issue_forward_move(self, reason): # 系统性误差：机器人没有严格旋转到位就进行下一步
        heading_rad = math.radians(self.robot.reading.pipeline_heading_degrees) + self.robot.pose.yaw # notice: radian or degree
        heading_rad = heading_rad % (2 * math.pi)  
        dx = math.cos(heading_rad) * self.forward_step
        dy = math.sin(heading_rad) * self.forward_step
        if self.start_second_search_turning:
            dx, dy, heading_rad = -dx, -dy, (heading_rad + math.pi) % (2 * math.pi)
            self.start_second_search_turning = False
        self.memory_readings.add_reading(RobotState(self.robot.pose, self.robot.reading))
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
        if not (self.workspace_min_x <= target_x <= self.workspace_max_x and self.workspace_min_y <= target_y <= self.workspace_max_y):
            if self.line_confirmed and self.start_second_search:
                self._complete(MissionState.COMPLETE, 'workspace boundary reached')
            elif self.line_confirmed and (not self.start_second_search):
                self.start_second_search = True
                self.start_second_search_turning = True # only turn once
                self._move_to_and_memorise(self.first_confirmed_measurement_pose.x, self.first_confirmed_measurement_pose.y, self.first_confirmed_measurement_pose.yaw)
                self._transition(MissionState.REACQUIRE, 'return to first confirmed measurement point')
                self.get_logger().info('return to first confirmed measurement point, and start the second search')
            else:
                self._complete(MissionState.FAILED, 'workspace boundary reached, but line is not confirmed')
            return

        
        # target_x = self._clamp(pose.x + dx, self.workspace_min_x, self.workspace_max_x)
        # target_y = self._clamp(pose.y + dy, self.workspace_min_y, self.workspace_max_y)

        if math.hypot(target_x - receiver_pose.x, target_y - receiver_pose.y) <= 1e-9:
            if self.line_confirmed:
                self._complete(MissionState.COMPLETE, 'workspace boundary reached')
            else:
                self._transition(MissionState.REACQUIRE, 'no available motion in workspace')
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
                self.robot.reading.signal_strength,
            )
        )

    def _move_to_and_memorise(self, x, y, yaw):
        receiver_pose = self.robot.robot_tf_to_receiver(self.robot.pose)
        goal_pose = RobotPose(x=x, y=y, z=receiver_pose.z, yaw=yaw)
        # self.get_logger().info("receiver move to (%.2f, %.2f), yaw = %.2f\nnow in (%.2f, %.2f), yaw = %.2f" % (x, y, yaw, receiver_pose.x, receiver_pose.y, receiver_pose.yaw))
        if self.robot.has_arrived_helpful_func(receiver_pose, goal_pose):
            self.get_logger().info("receiver has arrived before moving")
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
                self.robot.reading.signal_strength,
                note,
            )
        )
    
    def _record_measurement_point(self, note):
        if self.robot.pose is None or self.robot.reading is None:
            return
        if not(self.robot.reading.left_arrow and self.robot.reading.right_arrow):
            return
        self.measurement_track.append(
            step = self.step_count,
            state = self.state,
            pose = self.robot.pose,
            reading = self.robot.reading,
            note = note
        )


    def _complete(self, final_state, reason):
        self.done = True
        self.state = final_state
        self._record_log(reason)
        self.get_logger().info('Mission finished with state=%s: %s' % (final_state.value, reason))

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
