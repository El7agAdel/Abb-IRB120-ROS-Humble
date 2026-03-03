#!/usr/bin/env python3
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


ERROR_NAME_BY_CODE = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "PLAN_INVALIDATED",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


class EndEffectorGoalSender(Node):
    """Send one Cartesian end-effector goal through MoveIt and execute it."""

    def __init__(self) -> None:
        super().__init__("send_trajectory_ee_goal")
        self.startup_ok = False

        self.declare_parameter("action_name", "/move_action")
        self.declare_parameter("group_name", "irb120_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "link_6")
        self.declare_parameter("target_xyz", [0.1, -0.42, 0.0])
        self.declare_parameter("target_quat_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("constrain_orientation", False)
        self.declare_parameter("orientation_tolerance", 0.05)
        self.declare_parameter("allowed_planning_time", 6.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("max_velocity_scaling_factor", 0.25)
        self.declare_parameter("max_acceleration_scaling_factor", 0.25)
        self.declare_parameter("retry_on_failure", True)
        self.declare_parameter("wait_for_server_timeout_sec", 60.0)

        action_name = str(self.get_parameter("action_name").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.target_xyz = [float(v) for v in self.get_parameter("target_xyz").value]
        self.target_quat_xyzw = [
            float(v) for v in self.get_parameter("target_quat_xyzw").value
        ]
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.constrain_orientation = bool(
            self.get_parameter("constrain_orientation").value
        )
        self.orientation_tolerance = float(
            self.get_parameter("orientation_tolerance").value
        )
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
        self.retry_on_failure = bool(self.get_parameter("retry_on_failure").value)
        self.wait_for_server_timeout_sec = float(
            self.get_parameter("wait_for_server_timeout_sec").value
        )
        self._relaxed_retry_sent = False

        if len(self.target_xyz) != 3:
            raise ValueError("'target_xyz' must contain exactly 3 values.")
        if len(self.target_quat_xyzw) != 4:
            raise ValueError("'target_quat_xyzw' must contain exactly 4 values.")
        if self.position_tolerance <= 0.0:
            raise ValueError("'position_tolerance' must be > 0.")
        if self.orientation_tolerance <= 0.0:
            raise ValueError("'orientation_tolerance' must be > 0.")
        if self.target_xyz[1] > -0.15:
            self.get_logger().warn(
                "Requested target has y close to zero/positive in base_link. "
                "This robot's nominal reachable EE workspace is mostly at negative y "
                "(home is around y=-0.43), so IK may fail."
            )

        self._client = ActionClient(self, MoveGroup, action_name)
        self.get_logger().info(
            f"Waiting for MoveGroup action server: {action_name} "
            f"(timeout={self.wait_for_server_timeout_sec:.1f}s)"
        )
        if not self._client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec):
            self.get_logger().error(
                "MoveGroup action server is not available. "
                "Start MoveIt first with: "
                "'ros2 launch IRB120 moveit.launch.py' "
                "or launch with 'run_ee_goal_sender:=true'."
            )
            return

        self.startup_ok = True
        self._send_goal(relaxed=False)

    def _build_goal_constraints(
        self, target_xyz, position_tolerance: float, constrain_orientation: bool
    ) -> Constraints:
        target_pose = Pose()
        target_pose.position.x = target_xyz[0]
        target_pose.position.y = target_xyz[1]
        target_pose.position.z = target_xyz[2]
        target_pose.orientation.x = self.target_quat_xyzw[0]
        target_pose.orientation.y = self.target_quat_xyzw[1]
        target_pose.orientation.z = self.target_quat_xyzw[2]
        target_pose.orientation.w = self.target_quat_xyzw[3]

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]

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
        if constrain_orientation:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.ee_link
            orientation_constraint.orientation = target_pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints = [orientation_constraint]
        return constraints

    def _relaxed_target_xyz(self):
        return [
            max(-0.35, min(0.35, self.target_xyz[0])),
            min(self.target_xyz[1], -0.25),
            max(self.target_xyz[2], 0.10),
        ]

    def _send_goal(self, relaxed: bool) -> None:
        target_xyz = self.target_xyz if not relaxed else self._relaxed_target_xyz()
        position_tolerance = (
            self.position_tolerance if not relaxed else max(0.06, self.position_tolerance * 1.8)
        )
        constrain_orientation = self.constrain_orientation if not relaxed else False
        allowed_planning_time = (
            self.allowed_planning_time if not relaxed else max(10.0, self.allowed_planning_time * 2.0)
        )
        num_planning_attempts = (
            self.num_planning_attempts if not relaxed else max(20, self.num_planning_attempts * 2)
        )

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.goal_constraints = [
            self._build_goal_constraints(target_xyz, position_tolerance, constrain_orientation)
        ]
        goal.request.allowed_planning_time = allowed_planning_time
        goal.request.num_planning_attempts = num_planning_attempts
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal.request.start_state.is_diff = True

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(
            f"Sending EE goal ({'relaxed retry' if relaxed else 'initial'}): "
            f"group={self.group_name}, link={self.ee_link}, frame={self.base_frame}, "
            f"xyz={target_xyz}, quat_xyzw={self.target_quat_xyzw}, "
            f"position_tolerance={position_tolerance:.3f}, "
            f"allowed_planning_time={allowed_planning_time:.2f}, "
            f"num_planning_attempts={num_planning_attempts}, "
            f"constrain_orientation={constrain_orientation}"
        )
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        state: Optional[str] = None
        if hasattr(feedback, "state"):
            state = feedback.state
        if state:
            self.get_logger().debug(f"MoveGroup feedback state: {state}")

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("MoveGroup goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        result = future.result().result
        error_code_val = result.error_code.val
        error_code_name = ERROR_NAME_BY_CODE.get(error_code_val, "UNKNOWN")
        if error_code_val == 1:
            self.get_logger().info("MoveIt execution succeeded.")
        else:
            self.get_logger().error(
                "MoveIt execution failed with "
                f"MoveItErrorCodes={error_code_val} ({error_code_name})."
            )
            retryable_errors = (
                MoveItErrorCodes.FAILURE,
                MoveItErrorCodes.NO_IK_SOLUTION,
                MoveItErrorCodes.PLANNING_FAILED,
                MoveItErrorCodes.TIMED_OUT,
            )
            if (
                self.retry_on_failure
                and not self._relaxed_retry_sent
                and error_code_val in retryable_errors
            ):
                self._relaxed_retry_sent = True
                self.get_logger().warn(
                    "Retrying once with relaxed constraints for this 5-DOF arm "
                    "(no orientation constraint, larger position tolerance, "
                    "more planning time/attempts, and safer target z/y if needed)."
                )
                self._send_goal(relaxed=True)
                return
            if error_code_val in retryable_errors:
                self.get_logger().error(
                    "Try a closer target_xyz (for example '[0.0, -0.35, 0.18]'), "
                    "increase allowed_planning_time, and keep "
                    "constrain_orientation:=false for this 5-DOF arm."
                )
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = EndEffectorGoalSender()
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
