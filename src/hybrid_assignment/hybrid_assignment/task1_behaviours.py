#!/usr/bin/env python3
"""
task1_behaviours.py  —  STUDENT TASK 1

Goal: Implement the three leaf behaviours that the behavior tree will use.

A behaviour tree is made up of leaf nodes (Conditions and Actions) connected
by composite nodes (Sequence and Fallback). In this task you implement the
leaves. In Task 2 you connect them into a tree.

Each leaf inherits from py_trees.behaviour.Behaviour and must implement
the update() method, which returns one of:
    py_trees.common.Status.SUCCESS   — the behaviour completed successfully
    py_trees.common.Status.FAILURE   — the behaviour failed or condition is false
    py_trees.common.Status.RUNNING   — the behaviour is still in progress

The three behaviours you implement:

    1. IsObstacleTooClose  (Condition)
       Checks whether the LiDAR detects anything closer than a threshold
       in the forward arc of the robot.
       Returns SUCCESS if obstacle is too close (danger!).
       Returns FAILURE if the path ahead is clear.

    2. StopRobot  (Action)
       Publishes zero velocity to /cmd_vel to stop the robot immediately.
       Returns SUCCESS immediately after publishing.

    3. MoveToWaypoint  (Action)
       Drives the robot toward the current target waypoint using odometry.
       Returns RUNNING while the robot is still moving toward the waypoint.
       Returns SUCCESS when the robot is close enough to the waypoint.

How these connect to the hybrid architecture:
    - IsObstacleTooClose + StopRobot  →  the REACTIVE layer
      (fast, direct sensor-to-action, no planning)
    - MoveToWaypoint                  →  the DELIBERATIVE layer
      (goal-directed, uses odometry to reason about position)
"""

import math
import py_trees
import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# ── Tuning parameters ─────────────────────────────────────────
OBSTACLE_THRESHOLD = 0.4    # metres — stop if anything closer than this
FORWARD_ARC_DEG    = 30     # degrees each side of forward to check
WAYPOINT_TOLERANCE = 0.25   # metres — close enough to count as arrived
LINEAR_SPEED       = 0.15   # m/s — speed toward waypoint
ANGULAR_GAIN       = 1.5    # turning gain (higher = sharper turns)


# ─────────────────────────────────────────────────────────────
# Behaviour 1: IsObstacleTooClose  (Condition leaf)
#
# This is the reactive layer's sensor check.
# It reads the latest /scan message and checks the forward arc.
#
# The LiDAR gives 360 readings, one per degree (index 0 = forward,
# increasing counter-clockwise). The forward arc is roughly:
#   indices 0..FORWARD_ARC_DEG  and  360-FORWARD_ARC_DEG..359
#
# Returns SUCCESS  if any valid ray in that arc is < OBSTACLE_THRESHOLD
# Returns FAILURE  if the path ahead is clear (or no scan received yet)
# ─────────────────────────────────────────────────────────────
class IsObstacleTooClose(py_trees.behaviour.Behaviour):

    def __init__(self, node):
        super().__init__(name='IsObstacleTooClose')
        self.ros_node   = node
        self.latest_scan = None

        # Subscribe to the raw LiDAR topic
        self.ros_node.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def update(self):
        # TODO 1a: Check for obstacles in the forward arc
        #
        # 1. If self.latest_scan is None, return FAILURE (no data yet)
        #
        # 2. The scan has len(msg.ranges) readings.
        #    Work out how many indices correspond to FORWARD_ARC_DEG:
        #      total = len(self.latest_scan.ranges)
        #      arc   = int(FORWARD_ARC_DEG / 360 * total)
        #
        # 3. Check indices 0..arc (left of forward) and
        #                  total-arc..total (right of forward)
        #    For each index, check if the reading is valid and close:
        #      r = self.latest_scan.ranges[i]
        #      if not math.isinf(r) and r > 0.0 and r < OBSTACLE_THRESHOLD:
        #          return py_trees.common.Status.SUCCESS
        #
        # 4. If no close obstacle found, return FAILURE
        return py_trees.common.Status.FAILURE  # replace this


# ─────────────────────────────────────────────────────────────
# Behaviour 2: StopRobot  (Action leaf)
#
# This is the reactive layer's action.
# Publish zero velocity immediately and return SUCCESS.
# Simple and fast — no reasoning needed.
# ─────────────────────────────────────────────────────────────
class StopRobot(py_trees.behaviour.Behaviour):

    def __init__(self, node):
        super().__init__(name='StopRobot')
        self.ros_node = node
        self.publisher = self.ros_node.create_publisher(
            TwistStamped, '/cmd_vel', 10)

    def update(self):
        # TODO 1b: Publish zero velocity and return SUCCESS
        #
        # Create a TwistStamped message with all zeros and publish it.
        # Then return py_trees.common.Status.SUCCESS.
        #
        # Hint:
        #   msg = TwistStamped()
        #   msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        #   self.publisher.publish(msg)
        return py_trees.common.Status.SUCCESS  # replace with full implementation


# ─────────────────────────────────────────────────────────────
# Behaviour 3: MoveToWaypoint  (Action leaf)
#
# This is the deliberative layer.
# Drives the robot toward a list of waypoints in order.
# Uses odometry to know where the robot is.
#
# Logic:
#   - If we have arrived at the current waypoint → advance to next
#     and return SUCCESS (tree will re-tick next cycle)
#   - If all waypoints visited → return SUCCESS and stop
#   - Otherwise → compute direction to waypoint, publish velocity,
#     return RUNNING
# ─────────────────────────────────────────────────────────────
class MoveToWaypoint(py_trees.behaviour.Behaviour):

    def __init__(self, node, waypoints):
        """
        waypoints: list of (x, y) tuples in the Webots world frame.
        Example: [(1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]
        """
        super().__init__(name='MoveToWaypoint')
        self.ros_node        = node
        self.waypoints       = waypoints
        self.current_index   = 0
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.robot_yaw       = 0.0

        self.publisher = self.ros_node.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        self.ros_node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

    def _odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def update(self):
        # TODO 1c: Navigate to waypoints in sequence
        #
        # 1. If all waypoints are done (self.current_index >= len):
        #    Stop the robot and return SUCCESS.
        #
        # 2. Get the current target waypoint:
        #    target_x, target_y = self.waypoints[self.current_index]
        #
        # 3. Compute distance to waypoint:
        #    dx = target_x - self.robot_x
        #    dy = target_y - self.robot_y
        #    distance = math.sqrt(dx*dx + dy*dy)
        #
        # 4. If distance < WAYPOINT_TOLERANCE:
        #    Log that the waypoint was reached, advance self.current_index
        #    Return SUCCESS
        #
        # 5. Otherwise: compute angle to waypoint and turn toward it:
        #    angle_to_target = math.atan2(dy, dx)
        #    angle_error = angle_to_target - self.robot_yaw
        #    Normalise angle_error to [-pi, pi]:
        #      while angle_error >  math.pi: angle_error -= 2*math.pi
        #      while angle_error < -math.pi: angle_error += 2*math.pi
        #
        #    Publish velocity:
        #      msg.twist.linear.x  = LINEAR_SPEED
        #      msg.twist.angular.z = ANGULAR_GAIN * angle_error
        #    Return RUNNING

        return py_trees.common.Status.RUNNING  # replace with full implementation

    def _stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        self.publisher.publish(msg)
