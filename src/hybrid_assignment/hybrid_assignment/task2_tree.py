#!/usr/bin/env python3
"""
task2_tree.py  —  STUDENT TASK 2

Goal: Assemble the leaf behaviours from Task 1 into a complete behavior tree
      that demonstrates hybrid reactive-deliberative architecture.

The tree structure you will build:

    Root  [Fallback]
    ├── Reactive  [Sequence]          ← checked FIRST every tick
    │   ├── IsObstacleTooClose        ← Condition: is danger ahead?
    │   └── StopRobot                 ← Action: stop immediately
    └── Deliberative  [Sequence]      ← only runs if no obstacle
        └── MoveToWaypoint            ← Action: navigate to next waypoint

How a Fallback works:
    Tick children left to right.
    Return SUCCESS as soon as any child succeeds.
    Return FAILURE only if all children fail.

How a Sequence works:
    Tick children left to right.
    Return FAILURE as soon as any child fails.
    Return SUCCESS only if all children succeed.

Why this implements hybrid architecture:
    Every tick, the Fallback tries the Reactive branch first.
    - If IsObstacleTooClose returns SUCCESS → StopRobot runs → done.
      The deliberative branch never runs during danger.
    - If IsObstacleTooClose returns FAILURE (clear path) → Reactive Sequence
      fails → Fallback tries Deliberative branch → robot navigates.

    The reactive layer overrides the deliberative layer automatically,
    with no explicit priority code needed. This is the key property of
    behavior trees for hybrid architecture.

The tree is ticked at a fixed rate by a ROS2 timer.

Questions to answer in your report:
    Q1. Draw the behavior tree you implemented. Label each node as
        Condition, Action, Sequence, or Fallback.
    Q2. Place an obstacle in front of the robot while it is navigating.
        Which branch of the tree fires? Which branch is suppressed?
        Why does this happen automatically without any if/else code?
    Q3. What is the difference between the reactive and deliberative
        branches in terms of:
        (a) how fast they respond to new sensor data?
        (b) how much reasoning they do?
    Q4. What would you need to add to make the robot navigate AROUND
        the obstacle instead of just stopping?
    Q5. How does the hybrid architecture in your tree relate to the
        three-layer model (Strategic / Tactical / Reactive) from
        the lecture? Which layer is missing here, and what would it do?
"""

import rclpy
from rclpy.node import Node
import py_trees

from hybrid_assignment.task1_behaviours import (
    IsObstacleTooClose,
    StopRobot,
    MoveToWaypoint,
)

# ── Waypoints ─────────────────────────────────────────────────
# (x, y) coordinates in the Webots world frame.
# These are positions inside the break room — adjust if needed.
# The robot starts at approximately (0, 0).
WAYPOINTS = [
    ( 1.0,  0.0),   # move right
    ( 1.0,  1.5),   # move up
    ( 0.0,  1.5),   # move left
    ( 0.0,  0.0),   # return to start
]

# Tree tick rate
TICK_RATE_HZ = 10.0


class HybridBehaviourTree(Node):

    def __init__(self):
        super().__init__('hybrid_behaviour_tree')

        # ── TODO 2a: Create the leaf behaviours ───────────────
        # Instantiate the three behaviours from Task 1.
        # Each one needs 'self' (the ROS2 node) as its first argument.
        # MoveToWaypoint also needs the WAYPOINTS list.
        #
        # Example:
        #   obstacle_check = IsObstacleTooClose(self)
        #   stop           = StopRobot(self)
        #   navigate       = MoveToWaypoint(self, WAYPOINTS)

        # TODO: create the three behaviours here

        # ── TODO 2b: Build the reactive branch ───────────────
        # A Sequence that runs: check obstacle → stop robot
        # Use py_trees.composites.Sequence(name='...', memory=False)
        # Then call .add_children([...]) with the two behaviours.
        #
        # memory=False means the sequence re-evaluates from the start
        # every tick — important for reactive behaviour.

        # TODO: create reactive_branch = Sequence(...)

        # ── TODO 2c: Build the deliberative branch ────────────
        # A Sequence that runs: move to waypoint
        # Just one child for now — MoveToWaypoint handles the full
        # waypoint sequence internally.

        # TODO: create deliberative_branch = Sequence(...)

        # ── TODO 2d: Build the root Fallback ─────────────────
        # A Fallback with two children: reactive first, deliberative second.
        # Use py_trees.composites.Fallback(name='Root', memory=False)
        # Then call .add_children([reactive_branch, deliberative_branch])

        # TODO: create root = Fallback(...)

        # ── TODO 2e: Create and start the tree ───────────────
        # Create the behaviour tree and set up the tick timer.
        #
        # self.tree = py_trees.trees.BehaviourTree(root)
        # self.tree.setup(timeout=15)
        #
        # Create a timer that calls self._tick every 1/TICK_RATE_HZ seconds:
        # self.create_timer(1.0 / TICK_RATE_HZ, self._tick)

        # TODO: create and start the tree here

        self.get_logger().info(
            'HybridBehaviourTree started.\n'
            f'  Navigating {len(WAYPOINTS)} waypoints.\n'
            '  Place an obstacle in front of the robot to test\n'
            '  the reactive layer.'
        )

    def _tick(self):
        """Called every timer cycle — ticks the behaviour tree once."""
        # TODO 2f: Tick the tree and log the result
        #
        # self.tree.tick()
        #
        # Optional: print the tree state for debugging:
        # print(py_trees.display.unicode_tree(
        #     root=self.tree.root, show_status=True))
        pass


def main(args=None):
    rclpy.init(args=args)
    node = HybridBehaviourTree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
