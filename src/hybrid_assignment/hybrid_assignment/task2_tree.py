"""
task2_tree.py
==============
Assembles and ticks the hybrid reactive-deliberative patrol behavior tree.

Official docs to read before starting:
  Composites (Sequence, Selector/Fallback, memory parameter):
    https://py-trees.readthedocs.io/en/devel/composites.html
  Trees (BehaviourTree, setup, tick):
    https://py-trees.readthedocs.io/en/devel/trees.html
  Display (unicode_tree, show_status):
    https://py-trees.readthedocs.io/en/devel/display.html

py_trees version note:
  Run `python3 -c "import py_trees; print(dir(py_trees.composites))"`
  to see whether your installation provides Selector or Fallback.
  Both require memory=False in py_trees 2.x.
"""

import py_trees
import rclpy
from rclpy.node import Node

from hybrid_assignment.task1_behaviours import (
    DriveForward,
    IsObstacleTooClose,
    TurnAway,
)

TICK_RATE_HZ = 10.0


class PatrolTreeNode(Node):
    def __init__(self):
        super().__init__('patrol_behaviour_tree')

        # ------------------------------------------------------------------
        # TODO 2a — Instantiate the three leaf behaviours.
        #
        #   obstacle_check = IsObstacleTooClose(self)
        #   turn           = TurnAway(self)
        #   drive          = DriveForward(self)
        # ------------------------------------------------------------------

        # TODO 2a: replace this comment with the three instantiations above.

        # ------------------------------------------------------------------
        # TODO 2b — Build the reactive branch.
        #
        # A Sequence that checks for an obstacle THEN turns away from it.
        # Order matters: the condition must be the first child.
        #
        #   reactive_branch = py_trees.composites.Sequence(
        #       name='Reactive', memory=False,
        #       children=[obstacle_check, turn])
        # ------------------------------------------------------------------

        # TODO 2b: build reactive_branch here.

        # ------------------------------------------------------------------
        # TODO 2c — Build the deliberative branch.
        #
        # A Sequence that drives forward when the path is clear.
        #
        #   deliberative_branch = py_trees.composites.Sequence(
        #       name='Deliberative', memory=False,
        #       children=[drive])
        # ------------------------------------------------------------------

        # TODO 2c: build deliberative_branch here.

        # ------------------------------------------------------------------
        # TODO 2d — Build the root composite.
        #
        # A Selector (or Fallback) with reactive_branch as the FIRST child
        # and deliberative_branch as the second.
        #
        #   root = py_trees.composites.Selector(      # or Fallback
        #       name='Root', memory=False,
        #       children=[reactive_branch, deliberative_branch])
        # ------------------------------------------------------------------

        # TODO 2d: build root here.

        # ------------------------------------------------------------------
        # TODO 2e — Wrap the root in a BehaviourTree, call setup(), and
        #            create a timer that calls self._tick at TICK_RATE_HZ.
        #
        #   self.tree = py_trees.trees.BehaviourTree(root)
        #   self.tree.setup(timeout=15)
        #   self.create_timer(1.0 / TICK_RATE_HZ, self._tick)
        # ------------------------------------------------------------------

        # TODO 2e: create and start the tree here.

        # Print the tree structure once at startup.
        self.get_logger().info('PatrolTree started.')
        self.get_logger().info('Place an obstacle in Webots to test reactive layer.')
        # Uncomment once the tree is built:
        # self.get_logger().info(
        #     'Tree structure:\n' +
        #     py_trees.display.unicode_tree(root=root, show_status=False))

    def _tick(self):
        # ------------------------------------------------------------------
        # TODO 2f — Tick the tree and print its current state.
        #
        #   self.tree.tick()
        #   print(py_trees.display.unicode_tree(
        #       root=self.tree.root, show_status=True))
        # ------------------------------------------------------------------

        # TODO 2f: tick and display the tree here.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrolTreeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()