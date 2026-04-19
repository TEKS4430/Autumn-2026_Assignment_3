# Spring-2026 Task 3 — Hybrid Reactive-Deliberative Architecture

## Part A

### Overview

This assignment implements a **hybrid reactive-deliberative architecture**
using a behavior tree. The robot navigates to a sequence of waypoints
(deliberative layer) while reactively stopping when an obstacle is detected
(reactive layer). The two layers are composed automatically by the tree
structure — no explicit priority code is needed.

This connects to the lecture concept:
> "Real CPS typically use hybrid architectures: a fast reactive layer handles
> emergencies, while a deliberative layer handles goals and planning."

---

### Background Reading — Official py_trees Documentation

Before writing any code, read these sections of the official py_trees docs.
They are short and will save you significant debugging time.

| What you need to understand | Doc page |
|---|---|
| Status values (`SUCCESS`, `FAILURE`, `RUNNING`) | [Behaviours](https://py-trees.readthedocs.io/en/devel/behaviours.html) |
| Writing your own leaf node (`initialise`, `update`) | [Behaviours](https://py-trees.readthedocs.io/en/devel/behaviours.html) |
| `Sequence` and `Selector`/`Fallback` composites | [Composites](https://py-trees.readthedocs.io/en/devel/composites.html) |
| The `memory` parameter on composites | [Composites](https://py-trees.readthedocs.io/en/devel/composites.html) |
| `BehaviourTree`, `setup()`, `tick()` | [Trees](https://py-trees.readthedocs.io/en/devel/trees.html) |
| Printing the tree state to the console | [Display](https://py-trees.readthedocs.io/en/devel/display.html) |

> **py_trees version note — read this first.**
> The ROS 2 environment in this course ships with **py_trees 2.x**.
> In this version the OR-composite is called `Selector` in the library source,
> but `Fallback` is provided as an alias.
> Depending on the exact patch version installed, one name or the other may
> raise an `AttributeError`. To find out which name works on your machine:
> ```bash
> python3 -c "import py_trees; print(py_trees.__version__)"
> python3 -c "import py_trees; print(dir(py_trees.composites))"
> ```
> Use whichever of `Selector` or `Fallback` appears in that list.
> The `memory=False` keyword argument is **required** in 2.x — omitting it
> raises a `TypeError`. The [Composites](https://py-trees.readthedocs.io/en/devel/composites.html)
> page explains what `memory` controls.

---

### What is a Behavior Tree?

A behavior tree is made of nodes that each return one of three statuses:

| Status | Meaning |
|---|---|
| `SUCCESS` | The node completed successfully, or a condition is true |
| `FAILURE` | The node failed, or a condition is false |
| `RUNNING` | The node is still in progress (used by long-running actions) |

Two composite nodes connect leaf nodes into a tree:

**Sequence** — like AND. Ticks children left to right, stops on the first
`FAILURE`. Returns `SUCCESS` only if every child succeeds.

**Selector / Fallback** — like OR. Ticks children left to right, stops on the
first `SUCCESS`. Returns `FAILURE` only if every child fails.

The tree you will build:

```
Root  [Selector/Fallback]
├── Reactive  [Sequence]          ← checked FIRST every tick
│   ├── IsObstacleTooClose        ← Condition leaf
│   └── StopRobot                 ← Action leaf
└── Deliberative  [Sequence]      ← only runs when path is clear
    └── MoveToWaypoint            ← Action leaf
```

Every tick (10 times per second) the root Selector checks the Reactive branch
first. If an obstacle is detected `IsObstacleTooClose` returns `SUCCESS`, so
`StopRobot` also runs and the Reactive Sequence returns `SUCCESS` — the
Selector is satisfied and the Deliberative branch is **never ticked**.
If the path is clear `IsObstacleTooClose` returns `FAILURE`, the Reactive
Sequence fails, and the Selector moves on to the Deliberative branch.

---

### Setup

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

Open Webots and load the world file:
`src/hybrid_assignment/worlds/break_room.wbt`

Launch everything:
```bash
ros2 launch hybrid_assignment assignment.launch.py
```

The robot will start stationary. Once your behavior tree is implemented it
will begin navigating automatically.

---

### What Gets Launched

| Node | Purpose |
|---|---|
| `WebotsController` | Bridges Webots ↔ ROS 2. Publishes `/scan` and `/odom`. Subscribes to `/cmd_vel`. |
| `task2_tree` | **Your behavior tree node** — ticks the tree at 10 Hz |

---

### Topic Map

```
Webots
    ├── /scan   ← LaserScan  (sensor_msgs/LaserScan)   used by IsObstacleTooClose
    └── /odom   ← Odometry   (nav_msgs/Odometry)        used by MoveToWaypoint

Your behavior tree node
    └── /cmd_vel  → TwistStamped  (geometry_msgs/TwistStamped)  drives the robot
```

---

### Task 1 — Implement the Leaf Behaviours

**File:** `hybrid_assignment/task1_behaviours.py`

Each class inherits from `py_trees.behaviour.Behaviour`.
The only method you must fill in is `update()`, which is called every time
the node is ticked. It must return a `py_trees.common.Status` value.

See the [Behaviours](https://py-trees.readthedocs.io/en/devel/behaviours.html)
page for the full lifecycle (`setup`, `initialise`, `update`, `terminate`).
For this task you only need `update()`.

---

**TODO 1a — `IsObstacleTooClose.update()`**

This is a **Condition** node. It reads the latest LiDAR scan and checks
whether anything in the forward arc is closer than `OBSTACLE_THRESHOLD`.

- Return `SUCCESS` if an obstacle is detected (danger → stop).
- Return `FAILURE` if the path is clear (safe → navigate).

Hints:
- `self.latest_scan` holds the most recent `LaserScan` message; it is `None`
  until the first message arrives — handle that case or the node will crash.
- `self.latest_scan.ranges` is a list of float distances, one per degree.
  Index 0 is directly in front of the robot.
- Readings of `inf` mean "no return" (open space) — skip them.
- Check both the front-left arc (indices `0 … arc`) and the front-right arc
  (indices `total-arc … total`) to cover the full forward cone.
- The constants `FORWARD_ARC_DEG` and `OBSTACLE_THRESHOLD` are defined at
  the top of the file — use them, do not hard-code numbers.

---

**TODO 1b — `StopRobot.update()`**

This is an **Action** node. It publishes a zero-velocity command and returns
`SUCCESS` immediately.

Hints:
- Create a `TwistStamped` message. Its default field values are all zero,
  which is exactly what you want — you only need to set the timestamp.
- Use `self.ros_node.get_clock().now().to_msg()` for the stamp.
- Publish via `self.publisher`.
- This node should always return `SUCCESS`.

---

**TODO 1c — `MoveToWaypoint.update()`**

This is an **Action** node that drives the robot toward the current waypoint
using proportional heading control. It can return three different statuses:

- `RUNNING` — still driving toward the waypoint.
- `SUCCESS` — close enough; the waypoint is reached (advance the index).
- `FAILURE` — all waypoints have been visited (optional, the skeleton handles
  this with an early return).

Hints:

1. Guard against missing odometry — return `RUNNING` if `self.robot_x` has
   not been set yet.
2. Get the current target:
   ```python
   target_x, target_y = self.waypoints[self.current_index]
   ```
3. Compute straight-line distance. If it is less than `WAYPOINT_TOLERANCE`,
   increment `self.current_index` and return `SUCCESS`.
4. Otherwise compute the heading error and publish a velocity command, then
   return `RUNNING`.
5. Heading error formula:
   ```python
   angle_to_target = math.atan2(dy, dx)
   angle_error = angle_to_target - self.robot_yaw
   # Normalise to [-pi, pi] — this step is essential
   ```
   `math.atan2` is the right function here: it handles all four quadrants and
   returns a value in `[-pi, pi]`.
6. Set `msg.twist.linear.x = LINEAR_SPEED` and
   `msg.twist.angular.z = ANGULAR_GAIN * angle_error`.
   A positive `angle_error` means the target is to the left, so a positive
   angular velocity will turn the robot toward it.

---

### Task 2 — Build and Run the Behavior Tree

**File:** `hybrid_assignment/task2_tree.py`

Assemble the three leaf behaviours into the tree described above and start
ticking it. Read the
[Trees](https://py-trees.readthedocs.io/en/devel/trees.html) and
[Composites](https://py-trees.readthedocs.io/en/devel/composites.html)
pages before starting.

The `__init__` method of `BehaviourTreeNode` is where you build and start
the tree. The steps are broken into TODOs in the skeleton file.

**TODO 2a** — Instantiate the three leaf behaviours defined in `task1_behaviours.py`.

**TODO 2b** — Build the reactive branch: a `Sequence` containing
`obstacle_check` then `stop` (order matters — the condition must come first).

**TODO 2c** — Build the deliberative branch: a `Sequence` containing only
`navigate`.

**TODO 2d** — Build the root composite (`Selector` or `Fallback`, depending
on your py_trees version) containing `reactive_branch` then
`deliberative_branch`. The reactive branch **must** be the first child.

**TODO 2e** — Wrap the root in a `py_trees.trees.BehaviourTree`, call
`setup()` on it, then create a ROS 2 timer that calls `self._tick` at
`TICK_RATE_HZ`.

**TODO 2f** — Implement `_tick`: call `self.tree.tick()` and optionally print
the tree state using `py_trees.display.unicode_tree()`. See the
[Display](https://py-trees.readthedocs.io/en/devel/display.html) page for the
exact function signature — the `show_status` parameter is particularly useful.

---

### Testing

Once both files are implemented, launch and observe:

1. The robot should begin moving toward the first waypoint.
2. While it is moving, **drag any object into its path** in the Webots scene
   tree. The robot should stop within one tick (~100 ms).
3. Remove the object — the robot should resume navigating immediately.

The console tree printout shows which branch is active each tick:

```
Root [Selector]
├── Reactive [Sequence]        ← SUCCESS (obstacle present)
│   ├── IsObstacleTooClose ✓  SUCCESS
│   └── StopRobot ✓           SUCCESS
└── Deliberative [Sequence]    ← not ticked
    └── MoveToWaypoint         RUNNING
```

Common problems and fixes:

| Symptom | Likely cause |
|---|---|
| `AttributeError: module 'py_trees.composites' has no attribute 'Fallback'` | Use `Selector` instead — see version note above |
| `TypeError: __init__() missing keyword argument 'memory'` | Add `memory=False` to every composite constructor |
| Robot does not move at all | Check that `update()` in `MoveToWaypoint` is returning `RUNNING`, not `FAILURE` |
| Robot spins in place | Heading error not normalised to `[-pi, pi]` |
| `AttributeError: 'NoneType'` on first tick | Guard against `self.latest_scan` or `self.robot_x` being `None` |

---

### Questions for the Report

**Q1.** Draw the behavior tree you implemented. Label each node as
Condition, Action, Sequence, or Fallback/Selector. Show the status
(`SUCCESS` / `FAILURE` / `RUNNING`) of each node during (a) normal
navigation and (b) obstacle avoidance.

**Q2.** When you place an obstacle in front of the robot, which branch
fires and which is suppressed? Why does this happen without any explicit
`if/else` code in your tree assembly?

**Q3.** Compare the reactive and deliberative branches:
- How fast does each respond to new sensor data?
- How much reasoning does each one do?
- Which one could run safely on a real robot with a slow processor?

**Q4.** Currently the robot just stops when it detects an obstacle.
Sketch a modified behavior tree that would make the robot turn and
navigate around the obstacle. What new leaf nodes would you need?

**Q5.** The lecture describes a three-layer hybrid architecture:
Strategic → Tactical → Reactive. Your tree implements two of these.
Which layer is missing? What would it do in a more complete system?

---

### File Structure

```
hybrid_assignment/
├── hybrid_assignment/
│   ├── task1_behaviours.py   ← YOUR WORK: leaf behaviour classes
│   └── task2_tree.py         ← YOUR WORK: assemble and tick the tree
├── launch/
│   └── assignment.launch.py
├── worlds/
│   └── break_room.wbt
├── package.xml
└── setup.py
```

### Viewing Output

All output appears in the terminal where you ran `ros2 launch`.
Use `self.get_logger().info(...)` inside `BehaviourTreeNode` for ROS logging.
The tree visualisation in `_tick()` prints current node statuses every tick
and is the fastest way to debug which branch is active.

---

## Part B

Document how you used AI tools during this assignment. Include:
- Which tools you used and for what purpose.
- Any prompts that were particularly helpful.
- Cases where the AI was wrong or misleading, and how you detected that.

---

## Submission Instructions

Create a video covering Parts A and B:
- Demonstrate the environment running (Webots + ROS 2 launch).
- Show the robot navigating waypoints and stopping for an obstacle.
- Explain how you used AI in the assignment.