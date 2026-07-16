"""Stage-1 mission loop for directional peak validation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import atan2, cos, degrees, hypot, isfinite, radians, sin

from receiver_interface import ReceiverInterface, ReceiverPose, ReceiverReading
from robot_adapter import RobotInterface, RobotPose


class MissionState(str, Enum):
    SEARCH_PEAK = "search_peak"
    CENTER_ON_LINE = "center_on_line"
    MEASURE_ON_LINE = "measure_on_line"
    FOLLOW_LINE = "follow_line"
    REACQUIRE = "reacquire"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True)
class MissionLogEntry:
    """One observable step in the stage-1 mission loop."""

    step: int
    state: MissionState
    pose: RobotPose
    mode: str
    signal_strength: float
    compass_heading_degrees: float
    receiver_yaw_degrees: float
    alignment_error_degrees: float | None
    depth_m: float | None
    current_value: float | None
    arrow_hint: str | None
    note: str = ""


@dataclass(frozen=True)
class MissionResult:
    """Final mission outcome and recorded log."""

    final_state: MissionState
    steps_executed: int
    log: tuple[MissionLogEntry, ...]
    message: str = ""


class MinimalMissionController:
    """Directional search, peak-plus centering, and line-following controller."""

    def __init__(
        self,
        robot: RobotInterface,
        receiver: ReceiverInterface,
        workspace_min_x: float = 0.0,
        workspace_max_x: float = 10.0,
        workspace_min_y: float = -4.0,
        workspace_max_y: float = 4.0,
        search_probe_count: int = 12,
        search_probe_distance: float = 0.75,
        detect_threshold: float = 0.3,
        centering_step: float = 0.2,
        forward_step: float = 1.0,
        loss_threshold: float = 0.15,
        reacquire_probe_offset: float = 0.4,
        centered_timeout_per_area: float = 1.5,
        total_timeout_per_area: float = 6.0,
        initial_receiver_yaw_degrees: float | None = None,
        preferred_follow_heading_degrees: float | None = None,
        receiver_scan_count: int = 12,
        orientation_check_interval: int = 3,
        orientation_detect_threshold: float | None = None,
        bend_protection_angle_degrees: float = 30.0,
        bend_protection_steps: int = 4,
        repeat_center_grid_size: float = 0.3,
        repeat_center_limit: int = 4,
        local_loop_window: int = 12,
        local_loop_radius_m: float = 0.5,
        local_loop_min_path_m: float = 1.2,
    ) -> None:
        for name, value in (
            ("workspace_min_x", workspace_min_x),
            ("workspace_max_x", workspace_max_x),
            ("workspace_min_y", workspace_min_y),
            ("workspace_max_y", workspace_max_y),
            ("search_probe_distance", search_probe_distance),
            ("detect_threshold", detect_threshold),
            ("centering_step", centering_step),
            ("forward_step", forward_step),
            ("loss_threshold", loss_threshold),
            ("reacquire_probe_offset", reacquire_probe_offset),
            ("centered_timeout_per_area", centered_timeout_per_area),
            ("total_timeout_per_area", total_timeout_per_area),
            ("bend_protection_angle_degrees", bend_protection_angle_degrees),
            ("repeat_center_grid_size", repeat_center_grid_size),
            ("local_loop_radius_m", local_loop_radius_m),
            ("local_loop_min_path_m", local_loop_min_path_m),
        ):
            if not isfinite(value):
                raise ValueError(f"{name} must be finite.")
        if workspace_min_x > workspace_max_x:
            raise ValueError("workspace_min_x must not exceed workspace_max_x.")
        if workspace_min_y > workspace_max_y:
            raise ValueError("workspace_min_y must not exceed workspace_max_y.")
        if (
            not isinstance(search_probe_count, int)
            or isinstance(search_probe_count, bool)
            or search_probe_count < 3
        ):
            raise ValueError("search_probe_count must be an integer greater than or equal to 3.")
        if search_probe_distance <= 0.0:
            raise ValueError("search_probe_distance must be positive.")
        if detect_threshold < 0.0:
            raise ValueError("detect_threshold must be non-negative.")
        if orientation_detect_threshold is not None and (
            orientation_detect_threshold < 0.0 or not isfinite(orientation_detect_threshold)
        ):
            raise ValueError("orientation_detect_threshold must be a non-negative finite value.")
        if preferred_follow_heading_degrees is not None and not isfinite(
            preferred_follow_heading_degrees
        ):
            raise ValueError("preferred_follow_heading_degrees must be finite.")
        if centering_step <= 0.0:
            raise ValueError("centering_step must be positive.")
        if forward_step <= 0.0:
            raise ValueError("forward_step must be positive.")
        if loss_threshold < 0.0:
            raise ValueError("loss_threshold must be non-negative.")
        if reacquire_probe_offset < 0.0:
            raise ValueError("reacquire_probe_offset must be non-negative.")
        if centered_timeout_per_area <= 0.0:
            raise ValueError("centered_timeout_per_area must be positive.")
        if total_timeout_per_area <= 0.0:
            raise ValueError("total_timeout_per_area must be positive.")
        if (
            not isinstance(receiver_scan_count, int)
            or isinstance(receiver_scan_count, bool)
            or receiver_scan_count < 4
        ):
            raise ValueError("receiver_scan_count must be an integer greater than or equal to 4.")
        if (
            not isinstance(orientation_check_interval, int)
            or isinstance(orientation_check_interval, bool)
            or orientation_check_interval < 1
        ):
            raise ValueError("orientation_check_interval must be a positive integer.")
        if bend_protection_angle_degrees < 0.0:
            raise ValueError("bend_protection_angle_degrees must be non-negative.")
        if (
            not isinstance(bend_protection_steps, int)
            or isinstance(bend_protection_steps, bool)
            or bend_protection_steps < 0
        ):
            raise ValueError("bend_protection_steps must be a non-negative integer.")
        if repeat_center_grid_size <= 0.0:
            raise ValueError("repeat_center_grid_size must be positive.")
        if (
            not isinstance(repeat_center_limit, int)
            or isinstance(repeat_center_limit, bool)
            or repeat_center_limit < 1
        ):
            raise ValueError("repeat_center_limit must be a positive integer.")
        if (
            not isinstance(local_loop_window, int)
            or isinstance(local_loop_window, bool)
            or (local_loop_window != 0 and local_loop_window < 3)
        ):
            raise ValueError("local_loop_window must be 0 or an integer greater than or equal to 3.")
        if local_loop_radius_m <= 0.0:
            raise ValueError("local_loop_radius_m must be positive.")
        if local_loop_min_path_m < 0.0:
            raise ValueError("local_loop_min_path_m must be non-negative.")

        self._robot = robot
        self._receiver = receiver
        self._workspace_min_x = workspace_min_x
        self._workspace_max_x = workspace_max_x
        self._workspace_min_y = workspace_min_y
        self._workspace_max_y = workspace_max_y
        self._search_probe_count = search_probe_count
        self._search_probe_distance = search_probe_distance
        self._detect_threshold = detect_threshold
        self._orientation_detect_threshold = (
            detect_threshold * 0.5
            if orientation_detect_threshold is None
            else orientation_detect_threshold
        )
        self._centering_step = centering_step
        self._forward_step = forward_step
        self._loss_threshold = loss_threshold
        self._reacquire_probe_offset = reacquire_probe_offset
        self._workspace_area = (
            (self._workspace_max_x - self._workspace_min_x)
            * (self._workspace_max_y - self._workspace_min_y)
        )
        self._centered_timeout_steps = max(1, int(self._workspace_area * centered_timeout_per_area))
        self._total_timeout_steps = max(1, int(self._workspace_area * total_timeout_per_area))
        self._initial_receiver_yaw_degrees = (
            robot.pose.yaw_degrees
            if initial_receiver_yaw_degrees is None
            else initial_receiver_yaw_degrees
        ) % 360.0
        self._preferred_follow_heading_degrees = (
            None
            if preferred_follow_heading_degrees is None
            else preferred_follow_heading_degrees % 360.0
        )
        self._receiver_scan_count = receiver_scan_count
        self._orientation_check_interval = orientation_check_interval
        self._bend_protection_angle_degrees = bend_protection_angle_degrees
        self._bend_protection_steps = bend_protection_steps
        self._repeat_center_grid_size = repeat_center_grid_size
        self._repeat_center_limit = repeat_center_limit
        self._local_loop_window = local_loop_window
        self._local_loop_radius_m = local_loop_radius_m
        self._local_loop_min_path_m = local_loop_min_path_m
        self._reset_runtime_state()

    def _reset_runtime_state(self) -> None:
        self._line_confirmed = False
        self._first_line_pose: RobotPose | None = None
        self._tracking_sign: float | None = None
        self._last_peak_center_confirmed = False
        self._last_confirmed_line_heading: float | None = None
        self._steps_since_centered = 0
        self._search_heading: float | None = None
        self._search_last_signal: float | None = None
        self._receiver_yaw_degrees = self._initial_receiver_yaw_degrees
        self._follow_moves_since_orientation_scan = 0
        self._bend_protection_remaining = 0
        self._confirmed_center_visit_counts: dict[tuple[int, int], int] = {}
        self._last_repeat_center_key: tuple[int, int] | None = None
        self._confirmed_center_history: list[tuple[float, float]] = []
        self._local_loop_endpoint_detected = False

    def run(self, max_steps: int = 100) -> MissionResult:
        if not isinstance(max_steps, int) or isinstance(max_steps, bool) or max_steps <= 0:
            raise ValueError("max_steps must be a positive integer.")

        self._reset_runtime_state()
        state = MissionState.SEARCH_PEAK
        log: list[MissionLogEntry] = []
        effective_max_steps = min(max_steps, self._total_timeout_steps)
        completion_message = ""

        for step in range(1, effective_max_steps + 1):
            active_state = state
            try:
                if active_state == MissionState.SEARCH_PEAK:
                    reading, _ = self._search_peak_step()
                    note = "peak receiver scan"
                    if reading.signal_strength >= self._orientation_detect_threshold:
                        state = MissionState.CENTER_ON_LINE
                elif active_state == MissionState.CENTER_ON_LINE:
                    reading = self._center_on_line_step()
                    note = "peak cross-axis centering"
                    if self._last_peak_center_confirmed:
                        state = MissionState.MEASURE_ON_LINE
                    else:
                        state = MissionState.REACQUIRE
                elif active_state == MissionState.MEASURE_ON_LINE:
                    reading = self._measure_on_line_step()
                    note = "peak measurement"
                    state = MissionState.FOLLOW_LINE
                elif active_state == MissionState.FOLLOW_LINE:
                    reading = self._follow_line_step()
                    note = "peak follow and periodic cross-axis check"
                    if reading.signal_strength < self._loss_threshold:
                        state = MissionState.REACQUIRE
                    elif self._last_peak_center_confirmed:
                        state = MissionState.MEASURE_ON_LINE
                    elif not reading.valid_depth and not reading.valid_current:
                        state = MissionState.REACQUIRE
                    else:
                        state = MissionState.FOLLOW_LINE
                elif active_state == MissionState.REACQUIRE:
                    reading = self._reacquire_step()
                    note = "local peak cross-axis reacquire"
                    if self._last_peak_center_confirmed:
                        state = MissionState.CENTER_ON_LINE
                    elif self._has_completed_detectable_line_after_loss():
                        state = MissionState.COMPLETE
                else:
                    return MissionResult(active_state, step - 1, tuple(log), "")
            except RuntimeError as exc:
                failure_message = f"任务失败：{exc}"
                failure_reading = self._read_current_receiver_pose()
                log.append(
                    self._log_entry(step, MissionState.FAILED, failure_reading, failure_message)
                )
                return MissionResult(MissionState.FAILED, step, tuple(log), failure_message)

            if self._last_peak_center_confirmed or (
                reading.valid_depth and reading.signal_strength >= self._detect_threshold
            ):
                self._steps_since_centered = 0
            else:
                self._steps_since_centered += 1

            if active_state == MissionState.MEASURE_ON_LINE and reading.valid_depth:
                self._line_confirmed = True
                if self._first_line_pose is None:
                    self._first_line_pose = self._robot.pose

            if (
                self._line_confirmed
                and self._is_on_workspace_boundary(self._robot.pose)
                and self._has_progress_since_line_confirmation()
            ):
                state = MissionState.COMPLETE

            if self._has_repeat_center_endpoint():
                completion_message = "检测到端点附近局部重复：同一中心区域被重复命中。"
                note = completion_message
                state = MissionState.COMPLETE
            elif self._has_local_loop_endpoint():
                completion_message = "检测到端点附近局部打转：最近确认中心点长期集中在小区域。"
                note = completion_message
                state = MissionState.COMPLETE

            print(f"step: {step}", flush=True)
            log.append(self._log_entry(step, active_state, reading, note))
            if state == MissionState.COMPLETE:
                return MissionResult(state, step, tuple(log), completion_message)

            if self._steps_since_centered > self._centered_timeout_steps:
                failure_message = "区域划分不合理，请重新划分：长时间无法重新找到主线路。"
                log.append(self._log_entry(step, MissionState.FAILED, reading, failure_message))
                return MissionResult(MissionState.FAILED, step, tuple(log), failure_message)

        if effective_max_steps == self._total_timeout_steps:
            failure_message = "区域划分不合理，请重新划分：总工作时间超过当前区域阈值。"
        else:
            failure_message = "任务未在给定步数内完成。"
        if log:
            last_reading = self._read_current_receiver_pose()
            log.append(self._log_entry(effective_max_steps, MissionState.FAILED, last_reading, failure_message))
        return MissionResult(MissionState.FAILED, effective_max_steps, tuple(log), failure_message)

    def _search_peak_step(self) -> tuple[ReceiverReading, bool]:
        self._receiver.set_mode("peak") # 实际操作的时候得注意不能经常调
        base_pose = self._robot.pose
        base_reading = self._scan_receiver_orientation()
        if base_reading.signal_strength >= self._detect_threshold:
            self._search_last_signal = base_reading.signal_strength
            return base_reading, True

        if (
            self._search_heading is None
            or self._search_last_signal is None
            or base_reading.signal_strength < self._search_last_signal
        ):
            self._search_heading = self._select_search_advance_heading(base_reading)

        advanced_reading = self._advance_along_search_heading(base_pose)
        self._search_last_signal = advanced_reading.signal_strength
        return advanced_reading, False

    def _advance_along_search_heading(self, base_pose: RobotPose) -> ReceiverReading:
        if self._search_heading is None:
            raise RuntimeError("缺少峰值搜索方向。")

        next_x, next_y = self._bounded_target_from_heading(
            base_pose.x,
            base_pose.y,
            self._search_heading,
            self._search_probe_distance,
        )
        if abs(next_x - base_pose.x) <= 1e-9 and abs(next_y - base_pose.y) <= 1e-9:
            self._search_heading = None
            return self._scan_receiver_orientation()

        move_result = self._move_within_workspace(
            next_x,
            next_y,
            yaw_degrees=self._search_heading,
        )
        if not move_result.success:
            raise RuntimeError("峰值搜索前进动作落入不可通行区域。")
        return self._scan_receiver_orientation()

    def _center_on_line_step(self) -> ReceiverReading:
        self._receiver.set_mode("peak")
        return self._confirm_peak_center_by_cross_axis_search()

    def _measure_on_line_step(self) -> ReceiverReading:
        self._receiver.set_mode("peak")
        return self._read_aligned_to_compass()

    def _follow_line_step(self) -> ReceiverReading:
        self._receiver.set_mode("peak")
        self._last_peak_center_confirmed = False
        current = self._read_aligned_to_compass()  
        if not current.valid_depth and not current.valid_current:
            return current
        if self._should_start_bend_follow(current):
            self._start_bend_follow(current)
            next_x, next_y = self._forward_target(current.compass_heading_degrees)
            move_result = self._move_within_workspace(
                next_x,
                next_y,
                yaw_degrees=self._heading_between_points(
                    self._robot.pose.x,
                    self._robot.pose.y,
                    next_x,
                    next_y,
                    fallback=current.compass_heading_degrees,
                ),
            )
            if not move_result.success:
                raise RuntimeError("沿线跟踪移动落入不可通行区域。")
            self._follow_moves_since_orientation_scan = 1
            if self._bend_protection_remaining > 0:
                self._bend_protection_remaining -= 1
            return self._read_aligned_to_compass()
        orientation_check_interval = (
            1 if self._bend_protection_remaining > 0 else self._orientation_check_interval
        )
        if self._follow_moves_since_orientation_scan >= orientation_check_interval:
            self._follow_moves_since_orientation_scan = 0
            return self._confirm_peak_center_by_cross_axis_search()

        next_x, next_y = self._forward_target(current.compass_heading_degrees)
        move_result = self._move_within_workspace(
            next_x,
            next_y,
            yaw_degrees=self._heading_between_points(
                self._robot.pose.x,
                self._robot.pose.y,
                next_x,
                next_y,
                fallback=current.compass_heading_degrees,
            ),
        )
        if not move_result.success:
            raise RuntimeError("沿线跟踪移动落入不可通行区域。")
        self._follow_moves_since_orientation_scan += 1
        if self._bend_protection_remaining > 0:
            self._bend_protection_remaining -= 1
        return self._read_aligned_to_compass()

    def _reacquire_step(self) -> ReceiverReading:
        self._receiver.set_mode("peak")
        return self._confirm_peak_center_by_cross_axis_search()

    def _read_current_receiver_pose(self) -> ReceiverReading:
        return self._receiver.read(self._receiver_pose())

    def _read_with_receiver_yaw(self, receiver_yaw_degrees: float) -> ReceiverReading:
        self._rotate_receiver_to_yaw(receiver_yaw_degrees)
        return self._read_current_receiver_pose()

    def _rotate_receiver_to_yaw(self, receiver_yaw_degrees: float) -> None:
        self._receiver_yaw_degrees = receiver_yaw_degrees % 360.0
        rotation_result = self._robot.rotate_receiver(self._receiver_yaw_degrees)
        if not rotation_result.success:
            raise RuntimeError("接收机旋转动作失败。")

    def _read_aligned_to_compass(self) -> ReceiverReading:
        first = self._read_current_receiver_pose()
        return self._read_with_receiver_yaw(first.compass_heading_degrees)

    def _confirm_peak_center_by_cross_axis_search(self) -> ReceiverReading:
        self._last_peak_center_confirmed = False
        base_pose = self._robot.pose
        base_reading = self._scan_receiver_orientation()
        line_heading = base_reading.compass_heading_degrees

        offsets = self._cross_axis_offsets()
        samples: list[tuple[float, tuple[float, float], ReceiverReading]] = []
        seen_positions: set[tuple[float, float]] = set()

        for offset in offsets:
            candidate_x, candidate_y = self._offset_from_heading(
                base_pose.x,
                base_pose.y,
                line_heading,
                offset,
            )
            move_result = self._move_within_workspace(
                candidate_x,
                candidate_y,
                yaw_degrees=self._robot.pose.yaw_degrees,
            )
            if not move_result.success:
                continue

            position_key = (round(self._robot.pose.x, 9), round(self._robot.pose.y, 9))
            if position_key in seen_positions:
                continue
            seen_positions.add(position_key)

            reading = self._scan_receiver_orientation()
            samples.append(
                (
                    offset,
                    (self._robot.pose.x, self._robot.pose.y),
                    reading,
                )
            )

        if not samples:
            settle_result = self._move_within_workspace(
                base_pose.x,
                base_pose.y,
                yaw_degrees=base_pose.yaw_degrees,
            )
            if not settle_result.success:
                raise RuntimeError("横向峰值搜索后无法回到可通行位置。")
            return self._read_aligned_to_compass()

        best_index = max(range(len(samples)), key=lambda index: samples[index][2].signal_strength)
        if self._bend_protection_remaining > 0:
            best_index = self._select_peak_sample_index_for_bend(samples)
        best_offset, best_position, best_reading = samples[best_index]
        confirmed = self._is_confirmed_peak_sample(samples, best_index)

        settle_result = self._move_within_workspace(
            *best_position,
            yaw_degrees=self._robot.pose.yaw_degrees,
        )
        if not settle_result.success:
            raise RuntimeError("横向峰值搜索后无法停留在可通行位置。")

        final_reading = self._read_aligned_to_compass()
        confirmed = confirmed or final_reading.valid_depth or final_reading.valid_current
        self._last_peak_center_confirmed = confirmed
        if confirmed:
            self._record_confirmed_center()
            previous_heading = self._last_confirmed_line_heading
            self._last_confirmed_line_heading = final_reading.compass_heading_degrees
            self._receiver_yaw_degrees = final_reading.compass_heading_degrees % 360.0
            if (
                previous_heading is not None
                and self._axis_angle_difference(
                    previous_heading,
                    final_reading.compass_heading_degrees,
                )
                >= self._bend_protection_angle_degrees
            ):
                self._bend_protection_remaining = self._bend_protection_steps
                self._follow_moves_since_orientation_scan = 0
                self._tracking_sign = None
            return final_reading

        fallback_result = self._move_within_workspace(
            base_pose.x,
            base_pose.y,
            yaw_degrees=base_pose.yaw_degrees,
        )
        if not fallback_result.success:
            raise RuntimeError("横向峰值搜索失败后无法回到原位置。")
        return self._read_with_receiver_yaw(base_reading.receiver_yaw_degrees)

    def _should_start_bend_follow(self, reading: ReceiverReading) -> bool:
        if self._last_confirmed_line_heading is None:
            return False
        if not reading.valid_depth and not reading.valid_current:
            return False
        return (
            self._axis_angle_difference(
                self._last_confirmed_line_heading,
                reading.compass_heading_degrees,
            )
            >= self._bend_protection_angle_degrees
        )

    def _start_bend_follow(self, reading: ReceiverReading) -> None:
        self._bend_protection_remaining = max(self._bend_protection_steps, 1)
        self._follow_moves_since_orientation_scan = 0
        self._tracking_sign = None
        self._last_confirmed_line_heading = reading.compass_heading_degrees

    def _select_peak_sample_index_for_bend(
        self,
        samples: list[tuple[float, tuple[float, float], ReceiverReading]],
    ) -> int:
        if not samples:
            raise RuntimeError("横向峰值搜索缺少采样点。")

        offset_scale = max(self._search_probe_distance, self._reacquire_probe_offset, self._centering_step)
        best_index = 0
        best_score: tuple[float, float, float] | None = None

        for index, (offset, _, reading) in enumerate(samples):
            score = (
                reading.signal_strength
                - abs(offset) * (self._loss_threshold * 0.75 / offset_scale)
                + (0.03 if (reading.valid_depth or reading.valid_current) else 0.0),
                reading.signal_strength,
                -abs(offset),
            )
            if best_score is None or score > best_score:
                best_score = score
                best_index = index

        return best_index

    def _cross_axis_offsets(self) -> tuple[float, ...]:
        search_radius = max(
            self._search_probe_distance * 4.0,
            self._reacquire_probe_offset * 4.0,
            self._centering_step * 8.0,
        )
        steps = max(1, int(search_radius / self._centering_step))
        negative_offsets = tuple(-index * self._centering_step for index in range(steps, 0, -1))
        positive_offsets = tuple(index * self._centering_step for index in range(1, steps + 1))
        return negative_offsets + (0.0,) + positive_offsets

    def _is_confirmed_peak_sample(
        self,
        samples: list[tuple[float, tuple[float, float], ReceiverReading]],
        best_index: int,
    ) -> bool:
        best_reading = samples[best_index][2]
        if best_reading.signal_strength < self._loss_threshold:
            return False
        if best_reading.valid_depth or best_reading.valid_current:
            return True
        if best_index == 0 or best_index == len(samples) - 1:
            return False

        left_signal = samples[best_index - 1][2].signal_strength
        right_signal = samples[best_index + 1][2].signal_strength
        peak_margin = max(0.01, best_reading.signal_strength * 0.03)
        return (
            best_reading.signal_strength >= self._detect_threshold
            and best_reading.signal_strength - left_signal >= peak_margin
            and best_reading.signal_strength - right_signal >= peak_margin
        )

    def _scan_receiver_orientation(self) -> ReceiverReading:
        step_degrees = 360.0 / self._receiver_scan_count
        candidate_yaws = tuple(index * step_degrees for index in range(self._receiver_scan_count))
        best_reading: ReceiverReading | None = None
        best_score: tuple[float, float] | None = None
        for receiver_yaw_degrees in candidate_yaws:
            reading = self._read_with_receiver_yaw(receiver_yaw_degrees)  #等下看看这个是否合理
            alignment_score = (
                -reading.alignment_error_degrees
                if reading.alignment_error_degrees is not None
                else -180.0
            )
            score = (reading.signal_strength, alignment_score)
            if best_score is None or score > best_score:
                best_score = score
                best_reading = reading

        if best_reading is None:
            raise RuntimeError("接收机朝向扫描失败。")
        self._rotate_receiver_to_yaw(best_reading.receiver_yaw_degrees)
        return best_reading

    def _select_search_advance_heading(self, reading: ReceiverReading) -> float:
        heading = reading.compass_heading_degrees
        lateral_candidates = ((heading + 90.0) % 360.0, (heading - 90.0) % 360.0)
        viable_lateral = [
            candidate
            for candidate in lateral_candidates
            if self._available_distance_from_heading(candidate) > 1e-9
        ]
        if viable_lateral:
            return max(viable_lateral, key=self._available_distance_from_heading)

        line_candidates = (heading % 360.0, (heading + 180.0) % 360.0)
        return max(line_candidates, key=self._available_distance_from_heading)

    def _available_distance_from_heading(self, heading_degrees: float) -> float:
        heading_radians = radians(heading_degrees)
        return self._distance_to_workspace_boundary(
            self._robot.pose.x,
            self._robot.pose.y,
            cos(heading_radians),
            sin(heading_radians),
        )

    def _receiver_pose(self) -> ReceiverPose:
        pose = self._robot.pose
        return ReceiverPose(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            receiver_yaw_degrees=self._receiver_yaw_degrees,
        )

    def _forward_target(self, heading_degrees: float) -> tuple[float, float]:
        heading_radians = radians(heading_degrees)
        direction_x = cos(heading_radians)
        direction_y = sin(heading_radians)

        if abs(direction_x) < 1e-9 and abs(direction_y) < 1e-9:
            return self._robot.pose.x, self._robot.pose.y

        if self._tracking_sign is None:
            self._tracking_sign = self._select_tracking_sign(direction_x, direction_y)

        direction_x *= self._tracking_sign
        direction_y *= self._tracking_sign
        available_distance = self._distance_to_workspace_boundary(
            self._robot.pose.x,
            self._robot.pose.y,
            direction_x,
            direction_y,
        )

        step_scale = min(self._forward_step, available_distance)

        return (
            self._robot.pose.x + direction_x * step_scale,
            self._robot.pose.y + direction_y * step_scale,
        )

    def _select_tracking_sign(self, direction_x: float, direction_y: float) -> float:
        positive_distance = self._distance_to_workspace_boundary(
            self._robot.pose.x,
            self._robot.pose.y,
            direction_x,
            direction_y,
        )
        negative_distance = self._distance_to_workspace_boundary(
            self._robot.pose.x,
            self._robot.pose.y,
            -direction_x,
            -direction_y,
        )
        if self._preferred_follow_heading_degrees is None:
            return -1.0 if negative_distance > positive_distance else 1.0

        preferred_radians = radians(self._preferred_follow_heading_degrees)
        preferred_x = cos(preferred_radians)
        preferred_y = sin(preferred_radians)
        preferred_sign = 1.0 if direction_x * preferred_x + direction_y * preferred_y >= 0.0 else -1.0
        preferred_distance = positive_distance if preferred_sign > 0.0 else negative_distance
        opposite_distance = negative_distance if preferred_sign > 0.0 else positive_distance
        if preferred_distance > 1e-9 or opposite_distance <= 1e-9:
            return preferred_sign
        return -preferred_sign

    def _lateral_step(self, heading_degrees: float, arrow_hint: str) -> tuple[float, float]:
        heading_radians = radians(heading_degrees)
        left_x = -sin(heading_radians)
        left_y = cos(heading_radians)
        scale = self._centering_step if arrow_hint == "left" else -self._centering_step
        return left_x * scale, left_y * scale

    @staticmethod
    def _offset_from_heading(
        x: float,
        y: float,
        heading_degrees: float,
        offset: float,
    ) -> tuple[float, float]:
        heading_radians = radians(heading_degrees)
        left_x = -sin(heading_radians)
        left_y = cos(heading_radians)
        return x + left_x * offset, y + left_y * offset

    def _bounded_target_from_heading(
        self,
        x: float,
        y: float,
        heading_degrees: float,
        distance: float,
    ) -> tuple[float, float]:
        heading_radians = radians(heading_degrees)
        direction_x = cos(heading_radians)
        direction_y = sin(heading_radians)
        available_distance = self._distance_to_workspace_boundary(x, y, direction_x, direction_y)
        step_distance = min(distance, available_distance)
        return x + direction_x * step_distance, y + direction_y * step_distance

    @staticmethod
    def _heading_between_points(
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        fallback: float,
    ) -> float:
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        if abs(delta_x) <= 1e-9 and abs(delta_y) <= 1e-9:
            return fallback % 360.0
        return (degrees(atan2(delta_y, delta_x)) + 360.0) % 360.0

    def _move_within_workspace(self, x: float, y: float, yaw_degrees: float | None = None):
        return self._robot.move_to(
            self._clamp_x(x),
            self._clamp_y(y),
            yaw_degrees=yaw_degrees,
        )

    def _clamp_x(self, x: float) -> float:
        return min(max(x, self._workspace_min_x), self._workspace_max_x)

    def _clamp_y(self, y: float) -> float:
        return min(max(y, self._workspace_min_y), self._workspace_max_y)

    def _is_on_workspace_boundary(self, pose: RobotPose, tolerance: float = 1e-6) -> bool:
        return (
            abs(pose.x - self._workspace_min_x) <= tolerance
            or abs(pose.x - self._workspace_max_x) <= tolerance
            or abs(pose.y - self._workspace_min_y) <= tolerance
            or abs(pose.y - self._workspace_max_y) <= tolerance
        )

    def _distance_to_workspace_boundary(
        self,
        x: float,
        y: float,
        direction_x: float,
        direction_y: float,
    ) -> float:
        candidates: list[float] = []

        if direction_x > 1e-9:
            candidates.append((self._workspace_max_x - x) / direction_x)
        elif direction_x < -1e-9:
            candidates.append((self._workspace_min_x - x) / direction_x)

        if direction_y > 1e-9:
            candidates.append((self._workspace_max_y - y) / direction_y)
        elif direction_y < -1e-9:
            candidates.append((self._workspace_min_y - y) / direction_y)

        positive_candidates = [candidate for candidate in candidates if candidate >= 0.0]
        if not positive_candidates:
            return 0.0
        return min(positive_candidates)

    def _has_progress_since_line_confirmation(self) -> bool:
        if self._first_line_pose is None:
            return False

        delta_x = self._robot.pose.x - self._first_line_pose.x
        delta_y = self._robot.pose.y - self._first_line_pose.y
        return (delta_x * delta_x + delta_y * delta_y) ** 0.5 >= max(self._forward_step * 0.5, 1e-6)

    def _has_completed_detectable_line_after_loss(self) -> bool:
        return (
            self._bend_protection_remaining <= 0
            and self._line_confirmed
            and self._has_progress_since_line_confirmation()
        )

    def _record_confirmed_center(self) -> None:
        self._confirmed_center_history.append((self._robot.pose.x, self._robot.pose.y))
        key = self._center_visit_key(self._robot.pose)
        self._confirmed_center_visit_counts[key] = (
            self._confirmed_center_visit_counts.get(key, 0) + 1
        )
        if self._confirmed_center_visit_counts[key] > self._repeat_center_limit:
            self._last_repeat_center_key = key
        if self._is_recent_center_history_looping():
            self._local_loop_endpoint_detected = True

    def _has_repeat_center_endpoint(self) -> bool:
        return (
            self._last_repeat_center_key is not None
            and self._line_confirmed
            and self._has_progress_since_line_confirmation()
        )

    def _has_local_loop_endpoint(self) -> bool:
        return (
            self._local_loop_endpoint_detected
            and self._line_confirmed
            and self._has_progress_since_line_confirmation()
        )

    def _is_recent_center_history_looping(self) -> bool:
        if self._local_loop_window == 0:
            return False
        if len(self._confirmed_center_history) < self._local_loop_window:
            return False

        recent = self._confirmed_center_history[-self._local_loop_window :]
        center_x = sum(point[0] for point in recent) / len(recent)
        center_y = sum(point[1] for point in recent) / len(recent)
        max_radius = max(hypot(x - center_x, y - center_y) for x, y in recent)
        path_length = sum(
            hypot(current[0] - previous[0], current[1] - previous[1])
            for previous, current in zip(recent, recent[1:])
        )
        return (
            max_radius <= self._local_loop_radius_m
            and path_length >= self._local_loop_min_path_m
        )

    def _center_visit_key(self, pose: RobotPose) -> tuple[int, int]:
        return (
            round(pose.x / self._repeat_center_grid_size),
            round(pose.y / self._repeat_center_grid_size),
        )

    @staticmethod
    def _axis_angle_difference(first_degrees: float, second_degrees: float) -> float:
        difference = abs((first_degrees - second_degrees + 180.0) % 360.0 - 180.0)
        return min(difference, 180.0 - difference)

    def _log_entry(
        self,
        step: int,
        state: MissionState,
        reading: ReceiverReading,
        note: str,
    ) -> MissionLogEntry:
        return MissionLogEntry(
            step=step,
            state=state,
            pose=self._robot.pose,
            mode=reading.mode,
            signal_strength=reading.signal_strength,
            compass_heading_degrees=reading.compass_heading_degrees,
            receiver_yaw_degrees=reading.receiver_yaw_degrees,
            alignment_error_degrees=reading.alignment_error_degrees,
            depth_m=reading.depth_m,
            current_value=reading.current_value,
            arrow_hint=reading.arrow_hint,
            note=note,
        )
