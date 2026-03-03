#!/usr/bin/env python3
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint


ERROR_NAME_BY_CODE = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


class PickPlaceSender(Node):
    """Runs a pick/place sequence with optional gripper open/close steps."""

    def __init__(self) -> None:
        super().__init__("send_trajectory_pnp")
        self.startup_ok = False

        self.declare_parameter("action_name", "/move_action")
        self.declare_parameter("group_name", "irb120_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "link_6")

        self.declare_parameter("use_gripper", True)
        self.declare_parameter(
            "gripper_action_name", "/gripper_controller/follow_joint_trajectory"
        )
        self.declare_parameter("gripper_open", [0.035, 0.035])
        self.declare_parameter("gripper_closed", [0.002, 0.002])
        self.declare_parameter("gripper_motion_time_sec", 1.2)

        self.declare_parameter("cube_xyz", [0.6, 0.0, 0.3])
        self.declare_parameter("approach_above_top", 0.0)
        self.declare_parameter("pick_above_top", 0.0)
        self.declare_parameter("lift_height", 0.28)

        self.declare_parameter("place_xyz", [0.15, -0.25, 0.10])
        self.declare_parameter("place_above_top", 0.0)
        self.declare_parameter("joint6_name", "joint_6")
        self.declare_parameter("joint6_fixed_value", 0.0)
        self.declare_parameter("joint6_tolerance", 0.01)

        self.declare_parameter("position_tolerance", 0.025)
        self.declare_parameter("allowed_planning_time", 10.0)
        self.declare_parameter("num_planning_attempts", 20)
        self.declare_parameter("max_velocity_scaling_factor", 0.15)
        self.declare_parameter("max_acceleration_scaling_factor", 0.15)
        self.declare_parameter("wait_for_server_timeout_sec", 60.0)

        action_name = str(self.get_parameter("action_name").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)

        self.use_gripper = bool(self.get_parameter("use_gripper").value)
        self.gripper_action_name = str(self.get_parameter("gripper_action_name").value)
        self.gripper_open = [float(v) for v in self.get_parameter("gripper_open").value]
        self.gripper_closed = [float(v) for v in self.get_parameter("gripper_closed").value]
        self.gripper_motion_time_sec = float(
            self.get_parameter("gripper_motion_time_sec").value
        )

        self.cube_xyz = [float(v) for v in self.get_parameter("cube_xyz").value]
        self.approach_above_top = float(self.get_parameter("approach_above_top").value)
        self.pick_above_top = float(self.get_parameter("pick_above_top").value)
        self.lift_height = float(self.get_parameter("lift_height").value)

        self.place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]
        self.place_above_top = float(self.get_parameter("place_above_top").value)
        self.joint6_name = str(self.get_parameter("joint6_name").value)
        self.joint6_fixed_value = float(self.get_parameter("joint6_fixed_value").value)
        self.joint6_tolerance = float(self.get_parameter("joint6_tolerance").value)

        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.allowed_planning_time = float(
            self.get_parameter("allowed_planning_time").value
        )
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.max_velocity_scaling = float(
            self.get_parameter("max_velocity_scaling_factor").value
        )
        self.max_acceleration_scaling = float(
            self.get_parameter("max_acceleration_scaling_factor").value
        )
        self.wait_for_server_timeout_sec = float(
            self.get_parameter("wait_for_server_timeout_sec").value
        )

        self._validate_parameters()

        self._move_client = ActionClient(self, MoveGroup, action_name)
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, self.gripper_action_name
        )

        self.get_logger().info(
            f"Waiting for MoveGroup action server: {action_name} "
            f"(timeout={self.wait_for_server_timeout_sec:.1f}s)"
        )
        if not self._move_client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec):
            self.get_logger().error("MoveGroup action server is not available.")
            return

        if self.use_gripper:
            self.get_logger().info(
                f"Waiting for gripper action server: {self.gripper_action_name} "
                f"(timeout={self.wait_for_server_timeout_sec:.1f}s)"
            )
            if not self._gripper_client.wait_for_server(
                timeout_sec=self.wait_for_server_timeout_sec
            ):
                self.get_logger().warn(
                    "Gripper action server is not available. Running arm-only sequence."
                )
                self.use_gripper = False

        self._steps = self._build_steps()
        self._current_index = 0

        self.startup_ok = True
        self.get_logger().info(
            f"Starting pick/place sequence with {len(self._steps)} steps."
        )
        self._send_current_step()

    def _validate_parameters(self) -> None:
        if len(self.cube_xyz) != 3:
            raise ValueError("'cube_xyz' must contain exactly 3 values.")
        if len(self.place_xyz) != 3:
            raise ValueError("'place_xyz' must contain exactly 3 values.")
        if len(self.gripper_open) != 2 or len(self.gripper_closed) != 2:
            raise ValueError("'gripper_open' and 'gripper_closed' must contain 2 values.")
        if self.position_tolerance <= 0.0:
            raise ValueError("'position_tolerance' must be > 0.")
        if self.gripper_motion_time_sec <= 0.0:
            raise ValueError("'gripper_motion_time_sec' must be > 0.")

    def _build_steps(self) -> List[Dict]:
        cube_x, cube_y, cube_z = self.cube_xyz
        place_x, place_y, _ = self.place_xyz
        cube_middle = cube_z

        steps: List[Dict] = []
        if self.use_gripper:
            steps.append(
                {
                    "kind": "gripper",
                    "name": "open_before_pick",
                    "joints": self.gripper_open,
                }
            )

        steps.extend(
            [
                {
                    "kind": "move",
                    "name": "pre_pick",
                    "xyz": [cube_x, cube_y, cube_middle + self.approach_above_top],
                },
                {
                    "kind": "move",
                    "name": "pick",
                    "xyz": [cube_x, cube_y, cube_middle],
                },
            ]
        )

        if self.use_gripper:
            steps.append(
                {
                    "kind": "gripper",
                    "name": "close_on_cube",
                    "joints": self.gripper_closed,
                }
            )

        steps.extend(
            [
                {"kind": "move", "name": "lift", "xyz": [cube_x, cube_y, self.lift_height]},
                {
                    "kind": "move",
                    "name": "move_side",
                    "xyz": [place_x, place_y, self.lift_height],
                },
            ]
        )

        if self.use_gripper:
            steps.append(
                {"kind": "gripper", "name": "open_release", "joints": self.gripper_open}
            )

        steps.append({"kind": "move", "name": "retreat_side", "xyz": [place_x, place_y, self.lift_height]})
        return steps

    def _build_joint6_constraint(self) -> JointConstraint:
        joint6_constraint = JointConstraint()
        joint6_constraint.joint_name = self.joint6_name
        joint6_constraint.position = self.joint6_fixed_value
        joint6_constraint.tolerance_above = self.joint6_tolerance
        joint6_constraint.tolerance_below = self.joint6_tolerance
        joint6_constraint.weight = 1.0
        return joint6_constraint

    def _build_goal_constraints(self, target_xyz: List[float]) -> Constraints:
        target_pose = Pose()
        target_pose.position.x = target_xyz[0]
        target_pose.position.y = target_xyz[1]
        target_pose.position.z = target_xyz[2]
        target_pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]

        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [target_pose]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.ee_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        constraints.joint_constraints = [self._build_joint6_constraint()]
        return constraints

    def _send_current_step(self) -> None:
        if self._current_index >= len(self._steps):
            self.get_logger().info("Pick/place sequence completed.")
            rclpy.shutdown()
            return

        step = self._steps[self._current_index]
        if step["kind"] == "move":
            self._send_move_step(step)
            return
        self._send_gripper_step(step)

    def _send_move_step(self, step: Dict) -> None:
        target_xyz = [float(v) for v in step["xyz"]]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.goal_constraints = [self._build_goal_constraints(target_xyz)]
        goal.request.path_constraints.joint_constraints = [self._build_joint6_constraint()]
        goal.request.allowed_planning_time = self.allowed_planning_time
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal.request.start_state.is_diff = True

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(
            f"[{self._current_index + 1}/{len(self._steps)}] "
            f"{step['name']} -> {target_xyz}"
        )
        send_future = self._move_client.send_goal_async(goal)
        send_future.add_done_callback(self._move_goal_response_cb)

    def _send_gripper_step(self, step: Dict) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["gripper_finger_joint1", "gripper_finger_joint2"]

        point = JointTrajectoryPoint()
        point.positions = [float(step["joints"][0]), float(step["joints"][1])]
        point.time_from_start.sec = int(self.gripper_motion_time_sec)
        point.time_from_start.nanosec = int((self.gripper_motion_time_sec % 1.0) * 1e9)
        goal.trajectory.points = [point]

        self.get_logger().info(
            f"[{self._current_index + 1}/{len(self._steps)}] "
            f"{step['name']} -> joints={point.positions}"
        )
        send_future = self._gripper_client.send_goal_async(goal)
        send_future.add_done_callback(self._gripper_goal_response_cb)

    def _move_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup step rejected.")
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._move_result_cb)

    def _move_result_cb(self, future) -> None:
        result = future.result().result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            name = ERROR_NAME_BY_CODE.get(code, "UNKNOWN")
            failed_step = self._steps[self._current_index]["name"]
            self.get_logger().error(
                f"Move step '{failed_step}' failed with MoveItErrorCodes={code} ({name})."
            )
            rclpy.shutdown()
            return

        self._current_index += 1
        self._send_current_step()

    def _gripper_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper step rejected.")
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_cb)

    def _gripper_result_cb(self, future) -> None:
        result = future.result().result
        if result.error_code != 0:
            failed_step = self._steps[self._current_index]["name"]
            self.get_logger().error(
                f"Gripper step '{failed_step}' failed with error_code={result.error_code}."
            )
            rclpy.shutdown()
            return

        self._current_index += 1
        self._send_current_step()


def main() -> None:
    rclpy.init()
    node = PickPlaceSender()
    if not node.startup_ok:
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
