#!/usr/bin/env python3
"""Deplace le TCP du UR5e vers une pose cartesienne via MoveIt2.

Passe par l'action /move_action, comme le fait RViz. moveit_py n'etant
pas disponible sur Humble, on construit la MotionPlanRequest a la main.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

PLANNING_GROUP = "ur_manipulator"
BASE_FRAME = "base_link"
EEF_LINK = "tool0"


class MoveToPose(Node):
    def __init__(self):
        super().__init__("move_to_pose")
        self._client = ActionClient(self, MoveGroup, "move_action")

    def build_goal(self, pose: PoseStamped, pos_tol=0.01, ori_tol=0.05):
        """Construit un goal MoveGroup a partir d'une pose cartesienne."""
        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        # Contrainte de position : une petite sphere autour du point vise
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = EEF_LINK
        pc.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose.pose)

        # Contrainte d'orientation
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EEF_LINK
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        # planning_options.plan_only = False -> planifie ET execute
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        return goal

    def move_to(self, x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
        """Bloque jusqu'a la fin du mouvement. Retourne True si succes."""
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Serveur move_action indisponible")
            return False

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(f"Cible : ({x:.3f}, {y:.3f}, {z:.3f})")

        send = self._client.send_goal_async(self.build_goal(pose))
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()

        if not handle.accepted:
            self.get_logger().error("Goal refuse par MoveIt2")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val

        if code == 1:
            self.get_logger().info("Mouvement termine")
            return True

        self.get_logger().error(f"Echec, error_code = {code}")
        return False


def main():
    rclpy.init()
    node = MoveToPose()

    # Pince orientee vers le bas (rotation de pi autour de X)
    node.move_to(0.4, 0.2, 0.4)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
