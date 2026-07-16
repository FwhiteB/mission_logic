"""Fixed-rate ROS 2 publisher for simulated receiver readings."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

from field_sim import BiotSavartFieldModel, Pipeline, SimpleFieldModel
from pipeline_interfaces.msg import PlatformState, ReceiverReading as ReceiverReadingMsg
from receiver_interface import ReceiverPose
from receiver_sim import SimulatedReceiver


class ReceiverSimNode(Node):
    """Publish receiver readings from the latest platform state."""

    def __init__(self) -> None:
        super().__init__("receiver_sim_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("reading_rate_hz", 5.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_receiver_yaw_degrees", 0.0)

        config = self._load_config()
        pipeline = _build_pipeline(config)
        field_model = _build_field_model(config, pipeline)
        receiver_config = (
            config.get("receiver", {}) if isinstance(config.get("receiver", {}), dict) else {}
        )
        self._receiver = SimulatedReceiver(
            field_model,
            mode=receiver_config.get("mode", "peak"),
            frequency_hz=receiver_config.get("frequency_hz", 32768.0),
            noise_std=receiver_config.get("noise_std", 0.0),
            random_seed=receiver_config.get("random_seed"),
        )

        robot_config = config.get("robot", {}) if isinstance(config.get("robot", {}), dict) else {}
        controller_config = (
            config.get("controller", {}) if isinstance(config.get("controller", {}), dict) else {}
        )
        initial_receiver_yaw = controller_config.get(
            "initial_receiver_yaw_degrees",
            self.get_parameter("initial_receiver_yaw_degrees").value,
        )
        if initial_receiver_yaw is None:
            initial_receiver_yaw = robot_config.get("initial_yaw_degrees", 0.0)
        self._pose = ReceiverPose(
            x=float(robot_config.get("initial_x", self.get_parameter("initial_x").value)),
            y=float(robot_config.get("initial_y", self.get_parameter("initial_y").value)),
            z=float(self.get_parameter("initial_z").value),
            receiver_yaw_degrees=float(initial_receiver_yaw),
        )

        self._reading_pub = self.create_publisher(ReceiverReadingMsg, "/receiver/readings", 10)
        self.create_subscription(PlatformState, "/platform/state", self._on_platform_state, 10)
        reading_rate_hz = max(float(self.get_parameter("reading_rate_hz").value), 0.1)
        self.create_timer(1.0 / reading_rate_hz, self._publish_reading)

    def _load_config(self) -> dict:
        config_path = str(self.get_parameter("config_path").value)
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"config_path does not exist: {path}")
        raw = json.loads(path.read_text(encoding="utf-8-sig"))
        if not isinstance(raw, dict):
            raise ValueError("Top-level config must be a JSON object.")
        return raw

    def _on_platform_state(self, msg: PlatformState) -> None:
        self._pose = ReceiverPose(
            x=msg.x,
            y=msg.y,
            z=msg.z,
            receiver_yaw_degrees=msg.receiver_yaw_degrees,
        )

    def _publish_reading(self) -> None:
        reading = self._receiver.read(self._pose)
        msg = ReceiverReadingMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.x = reading.position.x
        msg.y = reading.position.y
        msg.z = reading.position.z
        msg.receiver_yaw_degrees = reading.receiver_yaw_degrees
        msg.signal_strength = reading.signal_strength
        msg.compass_heading_degrees = reading.compass_heading_degrees
        msg.has_alignment_error = reading.alignment_error_degrees is not None
        msg.alignment_error_degrees = reading.alignment_error_degrees or 0.0
        msg.valid_depth = reading.valid_depth
        msg.depth_m = reading.depth_m or 0.0
        msg.valid_current = reading.valid_current
        msg.current_value = reading.current_value or 0.0
        msg.has_arrow_hint = reading.arrow_hint is not None
        msg.arrow_hint = reading.arrow_hint or ""
        msg.mode = reading.mode
        msg.frequency_hz = reading.frequency_hz
        self._reading_pub.publish(msg)


def _build_pipeline(config: dict) -> Pipeline:
    pipeline_config = config.get("pipeline", {}) if isinstance(config.get("pipeline", {}), dict) else {}
    points = pipeline_config.get("points", [(0.0, 0.0, -1.0), (10.0, 0.0, -1.0)])
    return Pipeline.from_tuples(
        points=points,
        source_strength=pipeline_config.get("source_strength", 1.0),
        name=pipeline_config.get("name", "pipeline"),
    )


def _build_field_model(config: dict, pipeline: Pipeline):
    field_model_config = (
        config.get("field_model", {}) if isinstance(config.get("field_model", {}), dict) else {}
    )
    field_model_type = field_model_config.get("type", "simple")
    if field_model_type == "simple":
        return SimpleFieldModel(
            [pipeline],
            background=field_model_config.get("background", 0.0),
            distance_floor=field_model_config.get("distance_floor", 0.1),
            attenuation_power=field_model_config.get("attenuation_power", 2.0),
            current_scale=field_model_config.get("current_scale", 100.0),
        )
    if field_model_type == "biot_savart":
        return BiotSavartFieldModel(
            [pipeline],
            background=field_model_config.get("background", 0.0),
            current_scale=field_model_config.get("current_scale", 100.0),
            signal_scale=field_model_config.get("signal_scale", 2_000_000.0),
            singularity_floor=field_model_config.get("singularity_floor", 0.05),
            epsabs=field_model_config.get("epsabs", 1e-12),
            epsrel=field_model_config.get("epsrel", 1e-8),
        )
    raise ValueError("Config field 'field_model.type' must be either 'simple' or 'biot_savart'.")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ReceiverSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
