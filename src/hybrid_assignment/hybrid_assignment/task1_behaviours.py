"""
task1_behaviours.py
====================
Leaf behaviour classes for the hybrid reactive-deliberative assignment.

Official docs to read before starting:
  Behaviours (lifecycle, update, Status):
    https://py-trees.readthedocs.io/en/devel/behaviours.html
  Status values (SUCCESS / FAILURE / RUNNING):
    https://py-trees.readthedocs.io/en/devel/behaviours.html#status

Each class inherits from py_trees.behaviour.Behaviour and must implement
update() → py_trees.common.Status.

Do NOT import or call anything from task2_tree.py here.
"""

import math

import py_trees
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ---------------------------------------------------------------------------
# Tuning constants — adjust these if the robot behaves poorly, but understand
# what each one does before changing it.
# ---------------------------------------------------------------------------

OBSTACLE_THRESHOLD = 0.5   # metres — stop if anything is closer than this
FORWARD_ARC_DEG    = 30    # degrees — half-width of the forward danger cone
                           # (so the full cone is 2 × FORWARD_ARC_DEG wide)

WAYPOINT_TOLERANCE = 0.3   # metres — how close = "arrived"
LINEAR_SPEED       = 0.3   # m/s    — forward speed while navigating
ANGULAR_GAIN       = 1.5   # rad/s per rad of heading error (P-controller gain)


# ---------------------------------------------------------------------------
# Condition node
# ---------------------------------------------------------------------------

class IsObstacleTooClose(py_trees.behaviour.Behaviour):
    """
    Condition: is anything in the forward LiDAR arc closer than
    OBSTACLE_THRESHOLD metres?

    Returns
    -------
    SUCCESS  — obstacle detected (danger)
    FAILURE  — path is clear (safe to navigate)

    The ROS node is passed in so this behaviour can create its own
    subscriber without being a Node itself.
    """

    def __init__(self, ros_node):
        # Give the behaviour a human-readable name — it appears in the
        # console tree printout.
        super().__init__(name='IsObstacleTooClose')
        self.ros_node    = ros_node
        self.latest_scan = None   # populated by the subscriber callback

        # Subscribe to the LiDAR topic.
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
        # Step 3: Iterate over the front-left arc (indices 0 … arc-1) and
        #         the front-right arc (indices total-arc … total-1).
        #         Combine them in one loop with:
        #             list(range(arc)) + list(range(total - arc, total))
        #
        # Step 4: For each index, get the range value:
        #             r = self.latest_scan.ranges[i]
        #         Skip readings that are inf (open space) or <= 0 (invalid).
        #         If r < OBSTACLE_THRESHOLD → return Status.SUCCESS.
        #
        # Step 5: If the loop finishes without finding anything close,
        #         return Status.FAILURE.
        # ------------------------------------------------------------------

        raise NotImplementedError("TODO 1a: implement IsObstacleTooClose.update()")


# ---------------------------------------------------------------------------
# Action node — stop
# ---------------------------------------------------------------------------

class StopRobot(py_trees.behaviour.Behaviour):
    """
    Action: publish a zero-velocity TwistStamped and return SUCCESS.

    This node is always instant — it never returns RUNNING.
    """

    def __init__(self, ros_node):
        super().__init__(name='StopRobot')
        self.ros_node  = ros_node
        self.publisher = self.ros_node.create_publisher(
            TwistStamped, '/cmd_vel', 10)

    def update(self):
        # ------------------------------------------------------------------
        # TODO 1b — publish a zero-velocity command.
        #
        # Hint: A freshly constructed TwistStamped() already has all
        # velocity fields set to 0.0 — you only need to fill in the stamp
        # so downstream nodes know the message is current.
        #
        #     msg = TwistStamped()
        #     msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        #     self.publisher.publish(msg)
        #
        # Return Status.SUCCESS — stopping is always considered successful.
        # ------------------------------------------------------------------

        raise NotImplementedError("TODO 1b: implement StopRobot.update()")


# ---------------------------------------------------------------------------
# Action node — navigate
# ---------------------------------------------------------------------------

class MoveToWaypoint(py_trees.behaviour.Behaviour):
    """
    Action: drive the robot toward the current waypoint using a simple
    proportional heading controller fed by odometry.

    Returns
    -------
    RUNNING  — still driving toward the waypoint
    SUCCESS  — arrived (within WAYPOINT_TOLERANCE); index advanced
    FAILURE  — all waypoints have been visited

    Parameters
    ----------
    ros_node  : the ROS 2 node (provides clock, publisher, subscriber)
    waypoints : list of (x, y) tuples in the order they should be visited
    """

    def __init__(self, ros_node, waypoints):
        super().__init__(name='MoveToWaypoint')
        self.ros_node      = ros_node
        self.waypoints     = waypoints
        self.current_index = 0

        # Odometry state — updated by the subscriber callback below.
        # Start as None so update() can detect "not ready yet".
        self.robot_x   = None
        self.robot_y   = None
        self.robot_yaw = None

        self.publisher = self.ros_node.create_publisher(
            TwistStamped, '/cmd_vel', 10)

        self.ros_node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

    def _odom_callback(self, msg):
        """Extract (x, y, yaw) from the Odometry message."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convert the quaternion to a yaw angle.
        # The quaternion fields are (x, y, z, w).
        q = msg.pose.pose.orientation
        # yaw = atan2(2*(w*z + x*y),  1 - 2*(y² + z²))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def update(self):
        # ------------------------------------------------------------------
        # TODO 1c — implement waypoint navigation.
        #
        # Step 1: Guard — if odometry has not arrived yet (self.robot_x is
        #         None), return Status.RUNNING to wait quietly.
        #
        # Step 2: Guard — if all waypoints are done, return Status.FAILURE
        #         (the deliberative branch is finished).
        #         Condition: self.current_index >= len(self.waypoints)
        #
        # Step 3: Get the current target.
        #             target_x, target_y = self.waypoints[self.current_index]
        #
        # Step 4: Compute displacement and Euclidean distance.
        #             dx = target_x - self.robot_x
        #             dy = target_y - self.robot_y
        #             distance = math.sqrt(dx**2 + dy**2)
        #
        # Step 5: Arrival check.
        #         If distance < WAYPOINT_TOLERANCE:
        #             log a message, increment self.current_index,
        #             return Status.SUCCESS.
        #
        # Step 6: Compute heading error and drive.
        #
        #     angle_to_target = math.atan2(dy, dx)
        #     angle_error     = angle_to_target - self.robot_yaw
        #
        #     # Normalise to [-pi, pi] — without this the robot may spin
        #     # the long way around.
        #     while angle_error >  math.pi: angle_error -= 2 * math.pi
        #     while angle_error < -math.pi: angle_error += 2 * math.pi
        #
        #     msg = TwistStamped()
        #     msg.header.stamp        = self.ros_node.get_clock().now().to_msg()
        #     msg.twist.linear.x      = LINEAR_SPEED
        #     msg.twist.angular.z     = ANGULAR_GAIN * angle_error
        #     self.publisher.publish(msg)
        #
        #     return Status.RUNNING
        # ------------------------------------------------------------------

        raise NotImplementedError("TODO 1c: implement MoveToWaypoint.update()")