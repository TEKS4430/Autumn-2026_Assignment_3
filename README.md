# Spring-2026_Task_3

## Part A

### Overview
 
This assignment implements a **hybrid reactive-deliberative architecture**
using a behavior tree. The robot navigates to a sequence of waypoints
(deliberative layer) while reactively stopping when an obstacle is detected
(reactive layer). The two layers are composed automatically by the tree
structure — no explicit priority code is needed.
 
This connects concept of hybrid architecture:
> "Real CPS typically use hybrid architectures: a fast reactive layer handles
> emergencies, while a deliberative layer handles goals and planning."
 
---
 
### What is a Behavior Tree?
 
A behavior tree is made of nodes that each return one of three statuses:
 
| Status | Meaning |
|---|---|
| `SUCCESS` | The node completed successfully or a condition is true |
| `FAILURE` | The node failed or a condition is false |
| `RUNNING` | The node is still in progress (used by long actions) |
 
Two composite nodes connect leaf nodes into a tree:
 
**Sequence** — like AND. Ticks children left to right, stops on first FAILURE.
Returns SUCCESS only if all children succeed.
 
**Fallback** — like OR. Ticks children left to right, stops on first SUCCESS.
Returns FAILURE only if all children fail.
 
The tree you will build:
 
```
Root  [Fallback]
├── Reactive  [Sequence]          ← checked FIRST every tick
│   ├── IsObstacleTooClose        ← Condition
│   └── StopRobot                 ← Action
└── Deliberative  [Sequence]      ← only runs when path is clear
    └── MoveToWaypoint            ← Action
```
 
Every tick (10 times per second), the Fallback checks the Reactive branch
first. If an obstacle is detected, StopRobot runs and the Deliberative branch
never executes. If the path is clear, the Deliberative branch navigates toward
the next waypoint.
 
---
 
### Setup
 
 
```bash
cd /ros2_ws
colcon build 
source install/setup.bash
```
 
**Open Webots** with the world file under the src, break_room.wbt .
 
**Launch:**
 
```bash
ros2 launch hybrid_assignment assignment.launch.py
```
 
The robot will start stationary. Once your behavior tree is implemented,
it will begin navigating automatically.
 
---
 
### What Gets Launched
 
| Node | Purpose |
|---|---|
| `WebotsController` | Connects Webots to ROS2. Provides `/scan`, `/odom`. Accepts `/cmd_vel`. |
| `task2_tree` | **Your behavior tree node** — ticks the tree at 10 Hz |
 
---
 
### Topic Map
 
```
Webots
    ├── /scan   ← LiDAR readings (used by IsObstacleTooClose)
    └── /odom   ← Robot position (used by MoveToWaypoint)
 
Your behavior tree
    └── /cmd_vel  → commands the robot to move or stop
```
 
---
 
### Task 1 — Implement the Leaf Behaviours
 
**File:** `hybrid_assignment/task1_behaviours.py`
 
Implement three classes, each with an `update()` method that returns a
`py_trees.common.Status` value.
 
**TODO 1a — `IsObstacleTooClose.update()`**
 
Check the forward arc of the LiDAR scan for obstacles. Returns `SUCCESS`
if danger is detected, `FAILURE` if the path is clear.
 
```python
# The LiDAR has 360 readings (one per degree, index 0 = forward)
total = len(self.latest_scan.ranges)
arc   = int(FORWARD_ARC_DEG / 360 * total)
 
# Check front-left (0..arc) and front-right (total-arc..total)
for i in list(range(arc)) + list(range(total - arc, total)):
    r = self.latest_scan.ranges[i]
    if not math.isinf(r) and r > 0.0 and r < OBSTACLE_THRESHOLD:
        return py_trees.common.Status.SUCCESS  # obstacle!
 
return py_trees.common.Status.FAILURE  # clear
```
 
**TODO 1b — `StopRobot.update()`**
 
Publish a zero-velocity `TwistStamped` message and return `SUCCESS`.
 
```python
msg = TwistStamped()
msg.header.stamp = self.ros_node.get_clock().now().to_msg()
self.publisher.publish(msg)
return py_trees.common.Status.SUCCESS
```
 
**TODO 1c — `MoveToWaypoint.update()`**
 
Navigate toward `self.waypoints[self.current_index]` using odometry.
Return `RUNNING` while moving, `SUCCESS` when close enough, advance
to the next waypoint when one is reached.
 
The key steps:
1. Get current target: `target_x, target_y = self.waypoints[self.current_index]`
2. Compute distance: `distance = math.sqrt(dx²+dy²)`
3. If `distance < WAYPOINT_TOLERANCE` → advance waypoint index, return `SUCCESS`
4. Otherwise → compute heading error, publish velocity, return `RUNNING`
Heading error computation:
```python
angle_to_target = math.atan2(dy, dx)
angle_error = angle_to_target - self.robot_yaw
# Normalise to [-pi, pi]
while angle_error >  math.pi: angle_error -= 2 * math.pi
while angle_error < -math.pi: angle_error += 2 * math.pi
 
msg.twist.linear.x  = LINEAR_SPEED
msg.twist.angular.z = ANGULAR_GAIN * angle_error
```
 
---
 
### Task 2 — Build and Run the Behavior Tree
 
**File:** `hybrid_assignment/task2_tree.py`
 
Assemble the leaf behaviours into the tree and tick it.
 
**TODO 2a** — Instantiate the three behaviours:
```python
obstacle_check = IsObstacleTooClose(self)
stop           = StopRobot(self)
navigate       = MoveToWaypoint(self, WAYPOINTS)
```
 
**TODO 2b** — Build the reactive branch:
```python
reactive_branch = py_trees.composites.Sequence(
    name='Reactive', memory=False)
reactive_branch.add_children([obstacle_check, stop])
```
 
**TODO 2c** — Build the deliberative branch:
```python
deliberative_branch = py_trees.composites.Sequence(
    name='Deliberative', memory=False)
deliberative_branch.add_children([navigate])
```
 
**TODO 2d** — Build the root Fallback:
```python
root = py_trees.composites.Fallback(name='Root', memory=False)
root.add_children([reactive_branch, deliberative_branch])
```
 
**TODO 2e** — Create and start the tree:
```python
self.tree = py_trees.trees.BehaviourTree(root)
self.tree.setup(timeout=15)
self.create_timer(1.0 / TICK_RATE_HZ, self._tick)
```
 
**TODO 2f** — Tick the tree:
```python
def _tick(self):
    self.tree.tick()
    # Optional: visualise the tree state in the console
    print(py_trees.display.unicode_tree(
        root=self.tree.root, show_status=True))
```
 
---
 
### Testing
 
Once implemented, the robot should navigate the four waypoints in the
`WAYPOINTS` list. To test the reactive layer:
 
1. While the robot is moving, **place an object in front of it** in Webots
   (drag any furniture into its path in the scene tree).
2. The robot should **stop immediately**.
3. **Remove the object** — the robot should **resume navigating**.
You can watch which branch is active using the tree visualisation:
 
```
Root [Fallback]
├── Reactive [Sequence]        ← SUCCESS when obstacle present
│   ├── IsObstacleTooClose ✓  SUCCESS
│   └── StopRobot ✓           SUCCESS
└── Deliberative [Sequence]    ← not ticked (Fallback already succeeded)
    └── MoveToWaypoint         RUNNING
```
 
---
 
### Questions for the Report
 
**Q1.** Draw the behavior tree you implemented. Label each node as
Condition, Action, Sequence, or Fallback. Show the status of each
node during normal navigation and during obstacle avoidance.
 
**Q2.** When you place an obstacle in front of the robot, which branch
fires? Which branch is suppressed? Why does this happen without any
explicit `if/else` code in your tree assembly?
 
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
│   └── task2_tree.py         ← YOUR WORK: assemble and run the tree
├── launch/
│   └── assignment.launch.py
├── package.xml
└── setup.py
```
 
### Viewing Output
 
All output appears in the terminal where you ran `ros2 launch`.
Use `self.get_logger().info(...)` inside your node for logging.
The optional tree visualisation in `_tick()` prints the tree status
every tick — useful for debugging which branch is active.
 


## Part B

Report how you have used AI


## Returning instructions

- Create a video that reports Parts A and B, so demonstrates that you have got the environment up and running, implemented teleoperation, and reports how you have used AI in the assignment
