"""Small geometry helpers for field simulation."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt


@dataclass(frozen=True)
class Point3D:
    """A point or vector in meters."""

    x: float
    y: float
    z: float

    def __add__(self, other: "Point3D") -> "Point3D":
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z) 
 
    def __sub__(self, other: "Point3D") -> "Point3D":
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def scale(self, factor: float) -> "Point3D":
        return Point3D(self.x * factor, self.y * factor, self.z * factor)

    def dot(self, other: "Point3D") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Point3D") -> "Point3D":
        return Point3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def norm(self) -> float:
        return sqrt(self.dot(self))


def distance(a: Point3D, b: Point3D) -> float:
    """Return Euclidean distance between two 3D points."""

    return (a - b).norm()


def closest_point_on_segment(point: Point3D, start: Point3D, end: Point3D) -> Point3D:
    """Project a point onto a finite 3D segment."""

    segment = end - start
    length_sq = segment.dot(segment)  # Note: This is the squared length of the segment, not the actual length.
    if length_sq == 0.0:
        return start

    t = (point - start).dot(segment) / length_sq  # Project point onto the line defined by start and end, then clamp to [0, 1]
    t = max(0.0, min(1.0, t))
    return start + segment.scale(t)
