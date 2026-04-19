"""
task2_tree.py
=============
Assemble the leaf behaviours from task1_behaviours.py into a behavior tree
and tick it at a fixed rate from a ROS 2 node.

Official docs to read before starting:
  Composites (Sequence, Selector/Fallback, memory parameter):
    https://py-trees.readthedocs.io/en/devel/composites.html
  BehaviourTree and ticking:
    https://py-trees.readthedocs.io/en/devel/trees.html
  Printing tree state to the console:
    https://py-trees.readthedocs.io/en/devel/display.html

py_trees version note
---------------------
Run the following to check which OR-composite name exists on your machine:

    python3 -c "import py_trees; print(py_trees.__version__)"
    python3 -c "import py_trees; print(dir(py_trees.composites))"

In py_trees 2.x the class may be listed as 'Selector', 'Fallback', or both
(one is an alias for the other). Use whichever name appears.
The 'memory=False' keyword argument is REQUIRED in 2.x — see the Composites
docs for what it means and why it matters here.
"""

import py_trees
import rclpy
from rclpy.node import Node

from task1_behaviours import (
    IsObstacleTooClose,
    MoveToWaypoint,
    StopRobot,
)

# ---------------------------------------------------------------------------
# Waypoints — (x, y) in the Webots world frame.
# The robot visits them in order, then stops.
# Adjust these to suit your break_room.wbt layout.
# ---------------------------------------------------------------------------
WAYPOINTS = [
    ( 1.0,  0.0),
    ( -3.30,  4.0),
    (-5.0,  2.0),
    (-1.0,  0.0),
]

TICK_RATE_HZ = 10   # how many times per second the tree is ticked


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class BehaviourTreeNode(Node):
    """
    A ROS 2 node that owns the behavior tree and ticks it at TICK_RATE_HZ.

    The tree is built entirely inside __init__ — follow the TODO comments
    in order. After __init__ returns the tree is running; you do not need
    to change _tick().
    """

    def __init__(self):
        super().__init__('behaviour_tree_node')

        # ------------------------------------------------------------------
        # TODO 2a — Instantiate the three leaf behaviours.
        #
        # Each class is imported from task1_behaviours.  Pass `self` as the
        # ros_node argument so they can create subscribers and publishers.
        # MoveToWaypoint also needs the WAYPOINTS list.
        #
        # Hint:
        #     obstacle_check = IsObstacleTooClose(...)
        #     stop           = StopRobot(...)
        #     navigate       = MoveToWaypoint(...)
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # TODO 2b — Build the REACTIVE branch.
        #
        # This is a Sequence containing [obstacle_check, stop].
        # The condition MUST be first — if it returns FAILURE (clear path)
        # the Sequence stops immediately and StopRobot is never called.
        #
        # Required keyword: memory=False
        # See: https://py-trees.readthedocs.io/en/devel/composites.html
        #
        # Hint:
        #     reactive_branch = py_trees.composites.Sequence(
        #         name='Reactive', memory=False)
        #     reactive_branch.add_children([obstacle_check, stop])
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # TODO 2c — Build the DELIBERATIVE branch.
        #
        # This is also a Sequence, but it contains only [navigate].
        # Using a Sequence here (rather than adding navigate directly to the
        # root) makes it easy to add more deliberative steps later.
        #
        # Hint:
        #     deliberative_branch = py_trees.composites.Sequence(
        #         name='Deliberative', memory=False)
        #     deliberative_branch.add_children([navigate])
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # TODO 2d — Build the ROOT composite.
        #
        # This must be a Selector (or Fallback — same class, see version note
        # at the top of this file).  Its children are:
        #     [reactive_branch, deliberative_branch]
        # ORDER MATTERS: the reactive branch must be first so it is checked
        # before the deliberative branch on every tick.
        #
        # Hint:
        #     root = py_trees.composites.Selector(   # or Fallback
        #         name='Root', memory=False)
        #     root.add_children([reactive_branch, deliberative_branch])
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # TODO 2e — Wrap the root in a BehaviourTree, set it up, then
        #           create a ROS 2 timer to call self._tick at TICK_RATE_HZ.
        #
        # BehaviourTree.setup() calls setup() on every node in the tree.
        # It must be called before the first tick.
        #
        # Hint:
        #     self.tree = py_trees.trees.BehaviourTree(root)
        #     self.tree.setup(timeout=15)
        #     self.create_timer(1.0 / TICK_RATE_HZ, self._tick)
        # ------------------------------------------------------------------

        self.get_logger().info('Behaviour tree ready — starting to tick.')

    def _tick(self):
        # ------------------------------------------------------------------
        # TODO 2f — Tick the tree and (optionally) print its current state.
        #
        # self.tree.tick() advances the tree by one step.
        #
        # py_trees.display.unicode_tree() returns a multi-line string that
        # shows the tree structure and the status of each node after the tick.
        # The show_status=True argument adds ✓/✗/… symbols.
        # See: https://py-trees.readthedocs.io/en/devel/display.html
        #
        # Hint:
        #     self.tree.tick()
        #     print(py_trees.display.unicode_tree(
        #         root=self.tree.root, show_status=True))
        # ------------------------------------------------------------------

        raise NotImplementedError("TODO 2f: implement _tick()")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BehaviourTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()