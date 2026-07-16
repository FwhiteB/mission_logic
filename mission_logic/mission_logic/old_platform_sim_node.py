"""Ideal ROS 2 platform action server for the minimal mission loop."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from pipeline_interfaces.action import PlatformCommand
from pipeline_interfaces.msg import PlatformState
from terrain_sim import PlaneTerrain, SinusoidalTerrain


class PlatformSimNode(Node):
    """Execute one platform command at a time with ideal kinematics."""

    def __init__(self) -> None:
        super().__init__("platform_sim_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw_degrees", 0.0)
        self.declare_parameter("initial_receiver_yaw_degrees", 0.0)

        config = self._load_config()
        robot_config = config.get("robot", {}) if isinstance(config.get("robot", {}), dict) else {}
        self._terrain = _build_terrain(config)
        initial_x = float(robot_config.get("initial_x", self.get_parameter("initial_x").value))
        initial_y = float(robot_config.get("initial_y", self.get_parameter("initial_y").value))
        initial_yaw = float(
            robot_config.get(
                "initial_yaw_degrees",
                self.get_parameter("initial_yaw_degrees").value,
            )
        )
        controller_config = (
            config.get("controller", {}) if isinstance(config.get("controller", {}), dict) else {}
        )
        initial_receiver_yaw = controller_config.get(
            "initial_receiver_yaw_degrees",
            self.get_parameter("initial_receiver_yaw_degrees").value,
        )
        if initial_receiver_yaw is None:
            initial_receiver_yaw = initial_yaw

        sample = self._terrain.sample(initial_x, initial_y)
        self._x = initial_x
        self._y = initial_y
        self._z = sample.z
        self._yaw_degrees = initial_yaw % 360.0
        self._receiver_yaw_degrees = float(initial_receiver_yaw) % 360.0
        self._moving = False
        self._active_command = "idle"

        self._state_pub = self.create_publisher(PlatformState, "/platform/state", 10)
        state_rate_hz = max(float(self.get_parameter("state_rate_hz").value), 0.1)
        self.create_timer(1.0 / state_rate_hz, self._publish_state)
        self._action_server = ActionServer(
            self,
            PlatformCommand,
            "/platform/command",
            goal_callback=self._handle_goal,
            execute_callback=self._execute_command,
        )

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

    def _handle_goal(self, _goal_request):
        if self._moving:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_command(self, goal_handle):
        request = goal_handle.request
        self._moving = True
        self._active_command = _command_name(request.command_type)
        self._publish_feedback(goal_handle, 0.0, "started")

        success = True
        message = ""
        if request.command_type == PlatformCommand.Goal.MOVE_TO:
            success, message = self._move_to(request.target_x, request.target_y, request.yaw_degrees)
        elif request.command_type == PlatformCommand.Goal.MOVE_BY:
            success, message = self._move_to(
                self._x + request.delta_x,
                self._y + request.delta_y,
                request.yaw_degrees,
            )
        elif request.command_type == PlatformCommand.Goal.ROTATE_RECEIVER:
            self._receiver_yaw_degrees = request.receiver_yaw_degrees % 360.0
        elif request.command_type == PlatformCommand.Goal.HOLD:
            pass
        elif request.command_type == PlatformCommand.Goal.STOP:
            self._moving = False
        else:
            success = False
            message = f"Unsupported platform command type: {request.command_type}"

        self._moving = False
        self._active_command = "idle"
        self._publish_state()
        self._publish_feedback(goal_handle, 1.0, "complete" if success else message)

        result = PlatformCommand.Result()
        result.success = success
        result.final_state = self._state_msg()
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _move_to(self, x: float, y: float, yaw_degrees: float) -> tuple[bool, str]:
        sample = self._terrain.sample(float(x), float(y))
        if not sample.traversable:
            return False, "Requested motion ended on untraversable terrain."
        self._x = float(x)
        self._y = float(y)
        self._z = sample.z
        self._yaw_degrees = float(yaw_degrees) % 360.0
        return True, ""

    def _publish_feedback(self, goal_handle, progress: float, status: str) -> None:
        feedback = PlatformCommand.Feedback()
        feedback.current_state = self._state_msg()
        feedback.progress = progress
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    def _publish_state(self) -> None:
        self._state_pub.publish(self._state_msg())

    def _state_msg(self) -> PlatformState:
        msg = PlatformState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.x = self._x
        msg.y = self._y
        msg.z = self._z
        msg.yaw_degrees = self._yaw_degrees
        msg.receiver_yaw_degrees = self._receiver_yaw_degrees
        msg.moving = self._moving
        msg.active_command = self._active_command
        return msg


def _build_terrain(config: dict):
    terrain_config = config.get("terrain", {}) if isinstance(config.get("terrain", {}), dict) else {}
    terrain_type = terrain_config.get("type", "plane")
    if terrain_type == "plane":
        plane_config = (
            terrain_config.get("plane", {}) if isinstance(terrain_config.get("plane", {}), dict) else {}
        )
        return PlaneTerrain(
            slope_x=plane_config.get("slope_x", 0.0),
            slope_y=plane_config.get("slope_y", 0.0),
            offset=plane_config.get("offset", 0.0),
            max_slope_degrees=plane_config.get("max_slope_degrees", 25.0),
        )
    if terrain_type == "sinusoidal":
        sinusoidal_config = (
            terrain_config.get("sinusoidal", {})
            if isinstance(terrain_config.get("sinusoidal", {}), dict)
            else {}
        )
        return SinusoidalTerrain(
            amplitude=sinusoidal_config.get("amplitude", 0.2),
            wavelength_x=sinusoidal_config.get("wavelength_x", 5.0),
            wavelength_y=sinusoidal_config.get("wavelength_y", 5.0),
            base_height=sinusoidal_config.get("base_height", 0.0),
            max_slope_degrees=sinusoidal_config.get("max_slope_degrees", 25.0),
        )
    raise ValueError("Config field 'terrain.type' must be either 'plane' or 'sinusoidal'.")


def _command_name(command_type: int) -> str:
    names = {
        PlatformCommand.Goal.MOVE_TO: "move_to",
        PlatformCommand.Goal.MOVE_BY: "move_by",
        PlatformCommand.Goal.ROTATE_RECEIVER: "rotate_receiver",
        PlatformCommand.Goal.HOLD: "hold",
        PlatformCommand.Goal.STOP: "stop",
    }
    return names.get(command_type, "unknown")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlatformSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
