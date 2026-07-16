"""Minimal simulated robot boundary backed by the terrain model."""

from __future__ import annotations

import csv
import warnings
from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True)
class RobotPose:
    """Robot pose on the terrain surface."""

    x: float
    y: float
    z: float
    yaw_degrees: float = 0.0


@dataclass(frozen=True)
class RobotCommand:
    """A command sent from mission logic to the robot adapter."""

    sequence_id: int
    command_type: str
    target_x: float | None = None
    target_y: float | None = None
    target_z: float | None = None
    delta_x: float | None = None
    delta_y: float | None = None
    yaw_degrees: float | None = None
    speed_mps: float | None = None
    tolerance_m: float = 0.05
    reason: str = ""


@dataclass(frozen=True)
class MotionResult:
    """Result of a robot motion command."""

    success: bool
    pose: RobotPose
    traversable: bool
    command: RobotCommand | None = None
    start_pose: RobotPose | None = None
    requested_pose: RobotPose | None = None
    commanded_delta_x: float = 0.0
    commanded_delta_y: float = 0.0
    actual_delta_x: float = 0.0
    actual_delta_y: float = 0.0
    position_error_m: float = 0.0
    reached_target: bool = True
    message: str = ""


class RobotFeedbackError(RuntimeError):
    """Raised when robot feedback rows cannot produce a usable pose."""


class RobotInterface(Protocol):
    """Minimal robot API used by mission logic."""

    @property
    def pose(self) -> RobotPose:
        """Return the latest robot pose."""

    def move_to(self, x: float, y: float, yaw_degrees: float | None = None) -> MotionResult:
        """Move to an absolute horizontal target."""

    def move_by(self, dx: float, dy: float, yaw_degrees: float | None = None) -> MotionResult:
        """Move by a horizontal offset."""

    def rotate_receiver(self, receiver_yaw_degrees: float) -> MotionResult:
        """Rotate the receiver mount without moving the robot base."""

    def execute_command(self, command: RobotCommand) -> MotionResult:
        """Execute a structured command and return the reported actual motion."""


class SimpleRobot:
    """A small robot abstraction that snaps to the sampled terrain height."""

    def __init__(
        self,
        terrain,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw_degrees: float = 0.0,
        motion_distance_scale: float = 1.0,
    ) -> None:
        if motion_distance_scale <= 0.0 or not isfinite(motion_distance_scale):
            raise ValueError("motion_distance_scale must be a positive finite value.")

        self._terrain = terrain
        self._motion_distance_scale = motion_distance_scale
        self._next_sequence_id = 1
        initial_sample = terrain.sample(initial_x, initial_y)
        if not initial_sample.traversable:
            raise ValueError("Initial robot pose is not traversable.")

        self._pose = RobotPose(
            x=initial_x,
            y=initial_y,
            z=initial_sample.z,
            yaw_degrees=initial_yaw_degrees,
        )

    @property
    def pose(self) -> RobotPose:
        return self._pose

    def move_to(self, x: float, y: float, yaw_degrees: float | None = None) -> MotionResult:
        command = RobotCommand(
            sequence_id=self._allocate_sequence_id(),
            command_type="move_to",
            target_x=x,
            target_y=y,
            yaw_degrees=yaw_degrees,
        )
        return self.execute_command(command)

    def move_by(
        self,
        dx: float,
        dy: float,
        yaw_degrees: float | None = None,
    ) -> MotionResult:
        command = RobotCommand(
            sequence_id=self._allocate_sequence_id(),
            command_type="move_by",
            delta_x=dx,
            delta_y=dy,
            yaw_degrees=yaw_degrees,
        )
        return self.execute_command(command)

    def rotate_receiver(self, receiver_yaw_degrees: float) -> MotionResult:
        command = RobotCommand(
            sequence_id=self._allocate_sequence_id(),
            command_type="rotate_receiver",
            yaw_degrees=receiver_yaw_degrees,
        )
        return MotionResult(
            success=True,
            pose=self._pose,
            traversable=True,
            command=command,
            start_pose=self._pose,
            requested_pose=self._pose,
            reached_target=True,
        )

    def execute_command(self, command: RobotCommand) -> MotionResult:
        start_pose = self._pose
        requested_x, requested_y = self._requested_xy(command, start_pose)
        requested_yaw = (
            start_pose.yaw_degrees
            if command.yaw_degrees is None or command.command_type == "rotate_receiver"
            else command.yaw_degrees
        )

        requested_sample = self._terrain.sample(requested_x, requested_y)
        requested_pose = RobotPose(
            x=requested_x,
            y=requested_y,
            z=requested_sample.z,
            yaw_degrees=requested_yaw,
        )

        actual_x = start_pose.x + (requested_x - start_pose.x) * self._motion_distance_scale
        actual_y = start_pose.y + (requested_y - start_pose.y) * self._motion_distance_scale
        actual_sample = self._terrain.sample(actual_x, actual_y)
        reported_pose = RobotPose(
            x=actual_x,
            y=actual_y,
            z=actual_sample.z,
            yaw_degrees=requested_yaw,
        )
        if actual_sample.traversable:
            self._pose = reported_pose

        position_error_m = hypot(reported_pose.x - requested_x, reported_pose.y - requested_y)
        reached_target = actual_sample.traversable and position_error_m <= command.tolerance_m
        return MotionResult(
            success=actual_sample.traversable,
            pose=self._pose if actual_sample.traversable else reported_pose,
            traversable=actual_sample.traversable,
            command=command,
            start_pose=start_pose,
            requested_pose=requested_pose,
            commanded_delta_x=requested_x - start_pose.x,
            commanded_delta_y=requested_y - start_pose.y,
            actual_delta_x=reported_pose.x - start_pose.x,
            actual_delta_y=reported_pose.y - start_pose.y,
            position_error_m=position_error_m,
            reached_target=reached_target,
            message="" if actual_sample.traversable else "Requested motion ended on untraversable terrain.",
        )

    def _allocate_sequence_id(self) -> int:
        sequence_id = self._next_sequence_id
        self._next_sequence_id += 1
        return sequence_id

    @staticmethod
    def _requested_xy(command: RobotCommand, start_pose: RobotPose) -> tuple[float, float]:
        if command.command_type == "move_to":
            if command.target_x is None or command.target_y is None:
                raise ValueError("move_to command requires target_x and target_y.")
            return command.target_x, command.target_y
        if command.command_type == "move_by":
            if command.delta_x is None or command.delta_y is None:
                raise ValueError("move_by command requires delta_x and delta_y.")
            return start_pose.x + command.delta_x, start_pose.y + command.delta_y
        if command.command_type in {"rotate_receiver", "hold", "stop"}:
            return start_pose.x, start_pose.y
        raise ValueError(
            "command_type must be one of 'move_to', 'move_by', 'rotate_receiver', 'hold', or 'stop'."
        )


@dataclass(frozen=True)
class RobotFeedbackRow:
    """One row of real or replayed robot feedback."""

    x: float
    y: float
    z: float | None = None
    yaw_degrees: float | None = None
    success: bool = True
    traversable: bool | None = None
    sequence_id: int | None = None
    message: str = ""


class FeedbackLogRobot:
    """Robot adapter that replays actual robot feedback from tabular data."""

    def __init__(
        self,
        rows,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_z: float = 0.0,
        initial_yaw_degrees: float = 0.0,
    ) -> None:
        self._rows = tuple(rows)
        self._index = 0
        self._next_sequence_id = 1
        self._pose = RobotPose(
            x=initial_x,
            y=initial_y,
            z=initial_z,
            yaw_degrees=initial_yaw_degrees,
        )

    @classmethod
    def from_csv(
        cls,
        path: str | Path,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_z: float = 0.0,
        initial_yaw_degrees: float = 0.0,
    ) -> "FeedbackLogRobot":
        with Path(path).open(newline="", encoding="utf-8-sig") as file:
            rows = [cls.row_from_mapping(row) for row in csv.DictReader(file)]
        return cls(
            rows,
            initial_x=initial_x,
            initial_y=initial_y,
            initial_z=initial_z,
            initial_yaw_degrees=initial_yaw_degrees,
        )

    @staticmethod
    def row_from_mapping(row: dict[str, object]) -> RobotFeedbackRow:
        def pick(*names: str) -> object | None:
            for name in names:
                if name in row and row[name] not in (None, ""):
                    return row[name]
            return None

        return RobotFeedbackRow(
            x=_required_float(pick("实际_x", "反馈_x", "pose_x", "x"), "pose_x"),
            y=_required_float(pick("实际_y", "反馈_y", "pose_y", "y"), "pose_y"),
            z=_optional_float(pick("实际_z", "反馈_z", "pose_z", "z")),
            yaw_degrees=_optional_float(
                pick("机器狗yaw", "yaw_degrees", "robot_yaw", "yaw")
            ),
            success=_optional_bool(pick("成功", "success"), True),
            traversable=_optional_bool(pick("可通行", "traversable"), None),
            sequence_id=_optional_int(pick("sequence_id", "命令序号")),
            message=_optional_text(pick("message", "消息", "错误")),
        )

    @property
    def pose(self) -> RobotPose:
        return self._pose

    def move_to(self, x: float, y: float, yaw_degrees: float | None = None) -> MotionResult:
        command = RobotCommand(
            sequence_id=self._allocate_sequence_id(),
            command_type="move_to",
            target_x=x,
            target_y=y,
            yaw_degrees=yaw_degrees,
        )
        return self.execute_command(command)

    def move_by(
        self,
        dx: float,
        dy: float,
        yaw_degrees: float | None = None,
    ) -> MotionResult:
        command = RobotCommand(
            sequence_id=self._allocate_sequence_id(),
            command_type="move_by",
            delta_x=dx,
            delta_y=dy,
            yaw_degrees=yaw_degrees,
        )
        return self.execute_command(command)

    def rotate_receiver(self, receiver_yaw_degrees: float) -> MotionResult:
        command = RobotCommand(
            sequence_id=self._allocate_sequence_id(),
            command_type="rotate_receiver",
            yaw_degrees=receiver_yaw_degrees,
        )
        return MotionResult(
            success=True,
            pose=self._pose,
            traversable=True,
            command=command,
            start_pose=self._pose,
            requested_pose=self._pose,
            reached_target=True,
        )

    def execute_command(self, command: RobotCommand) -> MotionResult:
        if self._index >= len(self._rows):
            _warn_and_raise("No more robot feedback rows are available.")

        start_pose = self._pose
        requested_x, requested_y = SimpleRobot._requested_xy(command, start_pose)
        requested_yaw = start_pose.yaw_degrees if command.yaw_degrees is None else command.yaw_degrees
        requested_pose = RobotPose(
            x=requested_x,
            y=requested_y,
            z=command.target_z if command.target_z is not None else start_pose.z,
            yaw_degrees=requested_yaw,
        )

        row = self._rows[self._index]
        self._index += 1
        reported_pose = RobotPose(
            x=row.x,
            y=row.y,
            z=start_pose.z if row.z is None else row.z,
            yaw_degrees=requested_yaw if row.yaw_degrees is None else row.yaw_degrees,
        )
        self._pose = reported_pose

        if row.sequence_id is not None and row.sequence_id != command.sequence_id:
            warnings.warn(
                f"Robot feedback sequence_id {row.sequence_id} does not match command "
                f"{command.sequence_id}.",
                RuntimeWarning,
                stacklevel=2,
            )

        traversable = row.success if row.traversable is None else row.traversable
        position_error_m = hypot(reported_pose.x - requested_x, reported_pose.y - requested_y)
        return MotionResult(
            success=row.success,
            pose=reported_pose,
            traversable=traversable,
            command=command,
            start_pose=start_pose,
            requested_pose=requested_pose,
            commanded_delta_x=requested_x - start_pose.x,
            commanded_delta_y=requested_y - start_pose.y,
            actual_delta_x=reported_pose.x - start_pose.x,
            actual_delta_y=reported_pose.y - start_pose.y,
            position_error_m=position_error_m,
            reached_target=row.success and position_error_m <= command.tolerance_m,
            message=row.message,
        )

    def _allocate_sequence_id(self) -> int:
        sequence_id = self._next_sequence_id
        self._next_sequence_id += 1
        return sequence_id


def _required_float(value: object | None, field_name: str) -> float:
    parsed = _optional_float(value)
    if parsed is None or not isfinite(parsed):
        _warn_and_raise(f"缺少或无效的机器狗反馈字段: {field_name}")
    return parsed


def _optional_float(value: object | None) -> float | None:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    return float(text)


def _optional_int(value: object | None) -> int | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    return int(text)


def _optional_bool(value: object | None, default):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if not text:
        return default
    if text in {"true", "1", "yes", "y", "success", "成功", "是"}:
        return True
    if text in {"false", "0", "no", "n", "failed", "fail", "失败", "否"}:
        return False
    raise ValueError(f"Unsupported boolean value: {value!r}")


def _optional_text(value: object | None) -> str:
    if value is None:
        return ""
    return str(value).strip()


def _warn_and_raise(message: str):
    warnings.warn(message, RuntimeWarning, stacklevel=2)
    raise RobotFeedbackError(message)
