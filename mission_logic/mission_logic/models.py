"""Shared mission data models."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional
from dataclasses import field
from mission_logic.geometry import Point3D
from collections import deque


class MissionState(str, Enum):
    SEARCH_PEAK = "search_peak"
    CENTER_ON_LINE = "center_on_line"
    MEASURE_ON_LINE = "measure_on_line"
    FOLLOW_LINE = "follow_line"
    REACQUIRE = "reacquire"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class ReceiverReading:
    signal_strength: float
    depth: float
    current: float
    pipeline_heading_degrees: float
    signal_strength_percent: float
    left_arrow: bool
    right_arrow: bool
    stamp_sec: float
    frame_id: str

    magnetic_field: Optional[Point3D] = None




@dataclass(frozen=True)
class MoveResult:
    target: RobotPose
    reading: Optional[ReceiverReading] = None


@dataclass(frozen=True)
class MissionLogEntry:
    step: int
    state: MissionState
    pose: RobotPose
    reading: ReceiverReading
    note: str = ""


@dataclass(frozen=True)
class LineMeasurementPoint:
    index: int
    step: int
    state: MissionState

    pose: RobotPose
    reading: ReceiverReading

    note: str = ""

    @property
    def xy(self) -> tuple[float, float]:
        return self.pose.x, self.pose.y

    @property
    def estimated_pipeline_heading_rad(self) -> float:
        return self.pose.yaw + math.radians(self.reading.pipeline_heading_degrees)

    @property
    def centered(self) -> bool:
        return self.reading.left_arrow and self.reading.right_arrow


@dataclass(frozen=True)
class LineMeasurementTrack:
    points: list[LineMeasurementPoint] = field(default_factory=list)

    def append(
        self,
        step: int,
        state: MissionState,
        pose: RobotPose,
        reading: ReceiverReading,
        note: str = "",
    ):
        point = LineMeasurementPoint(
            index=len(self.points),
            step=step,
            state=state,
            pose=pose,
            reading=reading,
            note=note,
        )
        self.points.append(point)

    def as_xy_polyline(self) -> list[tuple[float, float]]:
        return [point.xy for point in self.points]

    def centered_points(self) -> list[LineMeasurementPoint]:
        return [point for point in self.points if point.centered]

@dataclass(frozen=True)
class RobotState:
    pose: RobotPose
    reading: ReceiverReading

class MemoryReadings:
    def __init__(self, memory_readings_capacity: int):
        self.readings = deque()
        self.memory_readings_capacity = memory_readings_capacity
    
    def add_reading(self, state: RobotState):
        self.readings.append(state)
        if len(self.readings) > self.memory_readings_capacity:
            self.readings.popleft()

    def get_oldest_reading(self):
        return self.readings[0] if self.readings else None

    def get_last_reading(self):
        return self.readings[-1] if self.readings else None

    