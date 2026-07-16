"""Simplified field model for buried pipeline detection."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees, hypot, isfinite, pi

from .geometry import Point3D, closest_point_on_segment, distance
from mission_logic.models import RobotPose


MU0 = 4.0 * pi * 1e-7


@dataclass(frozen=True)
class Pipeline:
    """A buried pipeline represented by a 3D polyline."""

    points: tuple[Point3D, ...]
    source_strength: float = 1.0
    name: str = "pipeline"

    def __post_init__(self) -> None:
        if len(self.points) < 2:
            raise ValueError("A pipeline must contain at least two points.")
        if self.source_strength <= 0.0 or not isfinite(self.source_strength):
            raise ValueError("source_strength must be a positive finite value.")

    @classmethod
    def from_tuples(
        cls,
        points: list[tuple[float, float, float]] | tuple[tuple[float, float, float], ...],
        source_strength: float = 1.0,
        name: str = "pipeline",
    ) -> "Pipeline":
        return cls(tuple(Point3D(*point) for point in points), source_strength, name)

    def closest_point(self, query: Point3D) -> Point3D:
        """Return the nearest point on the polyline."""

        closest, _ = self.closest_projection(query)
        return closest

    def closest_projection(self, query: Point3D) -> tuple[Point3D, Point3D]:
        """Return the nearest point and local segment tangent."""

        best_point: Point3D | None = None
        best_tangent: Point3D | None = None
        best_distance: float | None = None

        for start, end in zip(self.points, self.points[1:]):
            candidate = closest_point_on_segment(query, start, end)
            candidate_distance = distance(query, candidate)
            tangent = end - start
            tangent_norm = tangent.norm()
            if tangent_norm == 0.0:
                continue
            tangent = tangent.scale(1.0 / tangent_norm)
            if best_distance is None or candidate_distance < best_distance:
                best_point = candidate
                best_tangent = tangent
                best_distance = candidate_distance

        if best_point is None or best_tangent is None:
            raise ValueError("Pipeline contains no valid segment geometry.")
        return best_point, best_tangent

    def distance_to(self, query: Point3D) -> float:
        return distance(query, self.closest_point(query))


@dataclass(frozen=True)
class FieldMeasurement:
    """Internal ground-truth field reading at a receiver position."""

    position: Point3D
    signal_strength: float
    nearest_pipeline: str | None
    nearest_point: Point3D | None
    nearest_distance: float | None
    heading_degrees: float | None
    lateral_offset: float | None
    true_depth: float | None
    current_value: float | None
    magnetic_field: Point3D | None
    magnetic_x: float | None
    magnetic_y: float | None

class SimpleFieldModel:
    """Distance-based approximate field model.

    The model intentionally avoids high-fidelity electromagnetics. It gives a
    smooth and deterministic response strong enough to develop search,
    centering, and line-following logic for a straight-pipeline first draft.
    """

    def __init__(
        self,
        pipelines: list[Pipeline] | tuple[Pipeline, ...],
        background: float = 0.0,
        distance_floor: float = 0.1,
        attenuation_power: float = 2.0,
        current_scale: float = 100.0,
    ) -> None:
        if not pipelines:
            raise ValueError("At least one pipeline is required.")
        if distance_floor <= 0.0:
            raise ValueError("distance_floor must be positive.")
        if attenuation_power <= 0.0:
            raise ValueError("attenuation_power must be positive.")
        if current_scale <= 0.0:
            raise ValueError("current_scale must be positive.")

        self._pipelines = tuple(pipelines)
        self.background = background
        self.distance_floor = distance_floor
        self.attenuation_power = attenuation_power
        self.current_scale = current_scale

    @property
    def pipelines(self) -> tuple[Pipeline, ...]:
        return self._pipelines

    def sample(self, position: Point3D) -> FieldMeasurement:
        total_signal = self.background # 应该抛弃，改成下面两个使用矢量的
        total_signal_x = self.background
        total_signal_y = self.background
        nearest_pipeline: Pipeline | None = None
        nearest_point: Point3D | None = None
        nearest_distance: float | None = None
        nearest_tangent: Point3D | None = None

        for pipeline in self._pipelines:
            closest, tangent = pipeline.closest_projection(position)
            dist = distance(position, closest)
            effective_distance = max(dist, self.distance_floor)
            total_signal += pipeline.source_strength / (effective_distance**self.attenuation_power) # 应该改成下面这种

            # 计算磁场方向
            magnetic_tangent_x = tangent.y
            magnetic_tangent_y = - tangent.x
            distance_x = position.x - closest.x
            distance_y = position.y - closest.y
            vector_dot_product = magnetic_tangent_x * distance_x + magnetic_tangent_y * distance_y
            if vector_dot_product < 0:
                magnetic_tangent_x = -1 * magnetic_tangent_x
                magnetic_tangent_y = -1 * magnetic_tangent_y
            total_signal_x += (pipeline.source_strength * magnetic_tangent_x) / (effective_distance ** self.attenuation_power)
            total_signal_y += (pipeline.source_strength * magnetic_tangent_y) / (effective_distance ** self.attenuation_power)


            if nearest_distance is None or dist < nearest_distance:
                nearest_pipeline = pipeline
                nearest_point = closest
                nearest_distance = dist
                nearest_tangent = tangent

        if nearest_pipeline is None or nearest_point is None or nearest_tangent is None:
            return FieldMeasurement(
                position=position,
                signal_strength=total_signal,
                nearest_pipeline=None,
                nearest_point=None,
                nearest_distance=None,
                heading_degrees=None,
                lateral_offset=None,
                true_depth=None,
                current_value=None,
                magnetic_field = Point3D(total_signal_x, total_signal_y, 0.0),
                magnetic_x = total_signal_x,
                magnetic_y = total_signal_y
            ) 

        tangent_xy_norm = hypot(nearest_tangent.x, nearest_tangent.y)
        if tangent_xy_norm == 0.0:
            heading_degrees = 0.0
            left_normal = Point3D(0.0, 1.0, 0.0)
        else:
            heading_degrees = (degrees(atan2(nearest_tangent.y, nearest_tangent.x)) + 360.0) % 360.0
            left_normal = Point3D(
                -nearest_tangent.y / tangent_xy_norm,
                nearest_tangent.x / tangent_xy_norm,
                0.0,
            )

        offset_vector = Point3D(
            position.x - nearest_point.x,
            position.y - nearest_point.y,
            0.0,
        )
        lateral_offset = offset_vector.dot(left_normal)
        true_depth = abs(position.z - nearest_point.z)
        current_value = nearest_pipeline.source_strength * self.current_scale

        return FieldMeasurement(
            position=position,
            signal_strength=total_signal,
            nearest_pipeline=nearest_pipeline.name,
            nearest_point=nearest_point,
            nearest_distance=nearest_distance,
            heading_degrees=heading_degrees,
            lateral_offset=lateral_offset,
            true_depth=true_depth,
            current_value=current_value,
            magnetic_field=Point3D(total_signal_x, total_signal_y, 0.0),
            magnetic_x=total_signal_x,
            magnetic_y=total_signal_y
        )


class BiotSavartFieldModel:
    """Biot-Savart magnetic field model for polyline pipelines.

    Each pipeline segment is integrated with scipy.integrate.quad_vec and the
    resulting magnetic field vectors are superposed. The public measurement
    shape stays compatible with SimpleFieldModel so receiver simulation and
    mission logic do not need to know which field model is active.
    """

    def __init__(
        self,
        pipelines: list[Pipeline] | tuple[Pipeline, ...],
        background: float = 0.0,
        current_scale: float = 100.0,
        signal_scale: float = 2_000_000.0,
        singularity_floor: float = 0.05,
        epsabs: float = 1e-12,
        epsrel: float = 1e-8,
    ) -> None:
        if not pipelines:
            raise ValueError("At least one pipeline is required.")
        if current_scale <= 0.0:
            raise ValueError("current_scale must be positive.")
        if signal_scale <= 0.0:
            raise ValueError("signal_scale must be positive.")
        if singularity_floor <= 0.0:
            raise ValueError("singularity_floor must be positive.")
        if epsabs <= 0.0 or epsrel <= 0.0:
            raise ValueError("integration tolerances must be positive.")

        self._pipelines = tuple(pipelines)
        self.background = background
        self.current_scale = current_scale
        self.signal_scale = signal_scale
        self.singularity_floor = singularity_floor
        self.epsabs = epsabs
        self.epsrel = epsrel

    @property
    def pipelines(self) -> tuple[Pipeline, ...]:
        return self._pipelines

    def sample(self, position: RobotPose) -> FieldMeasurement:
        total_field = Point3D(0.0, 0.0, 0.0)
        nearest_pipeline: Pipeline | None = None
        nearest_point: Point3D | None = None
        nearest_distance: float | None = None
        nearest_tangent: Point3D | None = None

        for pipeline in self._pipelines:
            total_field = total_field + self._pipeline_field(position, pipeline)
            closest, tangent = pipeline.closest_projection(position)
            dist = distance(position, closest)
            if nearest_distance is None or dist < nearest_distance:
                nearest_pipeline = pipeline
                nearest_point = closest
                nearest_distance = dist
                nearest_tangent = tangent

        signal_strength = self.background + total_field.norm() * self.signal_scale

        if nearest_pipeline is None or nearest_point is None or nearest_tangent is None:
            return FieldMeasurement(
                position=position,
                signal_strength=signal_strength,
                nearest_pipeline=None,
                nearest_point=None,
                nearest_distance=None,
                heading_degrees=None,
                lateral_offset=None,
                true_depth=None,
                current_value=None,
                magnetic_field=total_field,
                magnetic_x=total_field.x,
                magnetic_y=total_field.y
            )

        heading_degrees, lateral_offset = _heading_and_lateral_offset(
            position,
            nearest_point,
            nearest_tangent,
        )
        true_depth = abs(position.z - nearest_point.z)
        current_value = nearest_pipeline.source_strength * self.current_scale

        return FieldMeasurement(
            position=position,
            signal_strength=signal_strength,
            nearest_pipeline=nearest_pipeline.name,
            nearest_point=nearest_point,
            nearest_distance=nearest_distance,
            heading_degrees=heading_degrees,
            lateral_offset=lateral_offset,
            true_depth=true_depth,
            current_value=current_value,
            magnetic_field=total_field,
            magnetic_x=total_field.x,
            magnetic_y=total_field.y
        )

    def _pipeline_field(self, position: Point3D, pipeline: Pipeline) -> Point3D:
        total = Point3D(0.0, 0.0, 0.0)
        for start, end in zip(pipeline.points, pipeline.points[1:]):
            total = total + self._segment_field(position, start, end, pipeline.source_strength)
        return total

    def _segment_field(
        self,
        position: Point3D,
        start: Point3D,
        end: Point3D,
        current_amps: float,
    ) -> Point3D:
        try:
            import numpy as np
            from scipy.integrate import quad_vec
        except ImportError as exc:
            raise RuntimeError(
                "BiotSavartFieldModel requires scipy. Install it with: python -m pip install scipy"
            ) from exc

        segment = end - start
        if segment.norm() == 0.0:
            return Point3D(0.0, 0.0, 0.0)

        coefficient = MU0 * current_amps / (4.0 * pi)

        def integrand(t: float):
            source = start + segment.scale(t)
            radius = position - source
            radius_norm = max(radius.norm(), self.singularity_floor)
            field = segment.cross(radius).scale(coefficient / (radius_norm**3))
            return np.array((field.x, field.y, field.z), dtype=float)

        result, _error = quad_vec(integrand, 0.0, 1.0, epsabs=self.epsabs, epsrel=self.epsrel)
        return Point3D(float(result[0]), float(result[1]), float(result[2]))


def _heading_and_lateral_offset(
    position: Point3D,
    nearest_point: Point3D,
    tangent: Point3D,
) -> tuple[float, float]:
    tangent_xy_norm = hypot(tangent.x, tangent.y)
    if tangent_xy_norm == 0.0:
        heading_degrees = 0.0
        left_normal = Point3D(0.0, 1.0, 0.0)
    else:
        heading_degrees = (degrees(atan2(tangent.y, tangent.x)) + 360.0) % 360.0
        left_normal = Point3D(
            -tangent.y / tangent_xy_norm,
            tangent.x / tangent_xy_norm,
            0.0,
        )

    offset_vector = Point3D(
        position.x - nearest_point.x,
        position.y - nearest_point.y,
        0.0,
    )
    lateral_offset = offset_vector.dot(left_normal)
    return heading_degrees, lateral_offset
