"""
task1_behaviours.py
====================
Leaf behaviour classes for the hybrid reactive-deliberative patrol assignment.

Official docs to read before starting:
  Behaviours (lifecycle, update, Status):
    https://py-trees.readthedocs.io/en/devel/behaviours.html
  Status values (SUCCESS / FAILURE / RUNNING):
    https://py-trees.readthedocs.io/en/devel/behaviours.html#status

YOUR WORK: implement update() in IsObstacleTooClose and TurnAway.
           DriveForward is pre-implemented — read and understand it.

Do NOT import or call anything from task2_tree.py here.
"""

import math

import py_trees
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

OBSTACLE_THRESHOLD = 0.5   # metres — react if anything is closer than this
FORWARD_ARC_DEG    = 30    # degrees — half-width of the forward danger cone

DRIVE_SPEED        = 0.3   # m/s   — forward speed when path is clear
TURN_SPEED         = 0.8   # rad/s — angular speed when avoiding an obstacle


# ---------------------------------------------------------------------------
# Condition node
# ---------------------------------------------------------------------------

class IsObstacleTooClose(py_trees.behaviour.Behaviour):
    """
    Condition: is anything in the forward LiDAR arc closer than
    OBSTACLE_THRESHOLD metres?

    Returns
    -------
    SUCCESS  — obstacle detected  (reactive branch takes over)
    FAILURE  — path is clear      (deliberative branch may run)
    """

    def __init__(self, ros_node):
        super().__init__(name='IsObstacleTooClose')
        self.ros_node    = ros_node
        self.latest_scan = None

        self.ros_node.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def update(self):
        # ------------------------------------------------------------------
        # TODO 1a — implement the obstacle check.
        #
        # Step 1: Guard — if no scan has arrived yet, return FAILURE so the
        #         robot does not falsely stop on startup.
        #
        # Step 2: Compute how many indices cover FORWARD_ARC_DEG degrees:
        #             total = len(self.latest_scan.ranges)
        #             arc   = int(FORWARD_ARC_DEG / 360 * total)
        #
        # Step 3: Combine the front-left and front-right index ranges:
        #             indices = list(range(arc)) + list(range(total - arc, total))
        #
        # Step 4: For each index check the range value:
        #             r = self.latest_scan.ranges[i]
        #         Skip readings that are inf (open space) or <= 0 (invalid).
        #         If r < OBSTACLE_THRESHOLD → return Status.SUCCESS immediately.
        #
        # Step 5: If the loop finishes without finding anything close,
        #         return Status.FAILURE.
        # ------------------------------------------------------------------

        raise NotImplementedError("TODO 1a: implement IsObstacleTooClose.update()")


# ---------------------------------------------------------------------------
# Action node — turn away from obstacle
# ---------------------------------------------------------------------------

class TurnAway(py_trees.behaviour.Behaviour):
    """
    Action: spin in place to avoid the obstacle ahead.

    Returns RUNNING every tick (the tree will keep calling this node as long
    as IsObstacleTooClose keeps returning SUCCESS).  Once the obstacle clears,
    IsObstacleTooClose returns FAILURE, the Reactive Sequence fails, and the
    Selector hands control to DriveForward — no explicit "done" logic needed.

    Returns
    -------
    RUNNING  — always (the Selector decides when to stop turning)
    """

    def __init__(self, ros_node):
        super().__init__(name='TurnAway')
        self.ros_node  = ros_node
        self.publisher = self.ros_node.create_publisher(
            TwistStamped, '/cmd_vel', 10)

    def update(self):
        # ------------------------------------------------------------------
        # TODO 1b — publish a turn-in-place command.
        #
        # Hints:
        #   msg = TwistStamped()
        #   msg.header.stamp        = self.ros_node.get_clock().now().to_msg()
        #   msg.twist.linear.x      = 0.0          ← stay still
        #   msg.twist.angular.z     = TURN_SPEED   ← spin left
        #   self.publisher.publish(msg)
        #
        # Return Status.RUNNING — the tree structure handles when to stop.
        # ------------------------------------------------------------------

        raise NotImplementedError("TODO 1b: implement TurnAway.update()")


# ---------------------------------------------------------------------------
# Action node — drive forward (PRE-IMPLEMENTED — do not modify)
# ---------------------------------------------------------------------------

class DriveForward(py_trees.behaviour.Behaviour):
    """
    Action: drive straight ahead at DRIVE_SPEED.

    This node runs only when IsObstacleTooClose returns FAILURE (path clear).
    It always returns RUNNING — the robot keeps driving until an obstacle
    triggers the reactive branch again.

    Returns
    -------
    RUNNING  — always
    """

    def __init__(self, ros_node):
        super().__init__(name='DriveForward')
        self.ros_node  = ros_node
        self.publisher = self.ros_node.create_publisher(
            TwistStamped, '/cmd_vel', 10)

    def update(self):
        msg = TwistStamped()
        msg.header.stamp    = self.ros_node.get_clock().now().to_msg()
        msg.twist.linear.x  = DRIVE_SPEED
        msg.twist.angular.z = 0.0
        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING