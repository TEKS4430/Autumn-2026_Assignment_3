# Autumn-2026 Task 3 — Hybrid Reactive-Deliberative Architecture
 
## Part A: Studying and technical implementation — 4 points
 
### Overview
 
This assignment implements a **hybrid reactive-deliberative architecture**
using a behavior tree. The robot patrols its environment by driving forward
continuously. When its LiDAR detects an obstacle, the reactive layer
interrupts forward motion and spins the robot until the path is clear.
 
This connects to the concept:
> "Real CPS typically use hybrid architectures: a fast reactive layer handles
> emergencies, while a deliberative layer handles goals and planning."
 
---
 
### Background Reading — Official py_trees Documentation
 
Before writing any code, read these sections of the official py_trees docs.
 
| What you need to understand | Doc page |
|---|---|
| Status values (`SUCCESS`, `FAILURE`, `RUNNING`) | [Behaviours](https://py-trees.readthedocs.io/en/devel/behaviours.html) |
| Writing your own leaf node (`initialise`, `update`) | [Behaviours](https://py-trees.readthedocs.io/en/devel/behaviours.html) |
| `Sequence` and `Selector`/`Fallback` composites | [Composites](https://py-trees.readthedocs.io/en/devel/composites.html) |
| The `memory` parameter on composites | [Composites](https://py-trees.readthedocs.io/en/devel/composites.html) |
| `BehaviourTree`, `setup()`, `tick()` | [Trees](https://py-trees.readthedocs.io/en/devel/trees.html) |
| Printing the tree state to the console | [Display](https://py-trees.readthedocs.io/en/devel/display.html) |
 

 
### What is a Behavior Tree?
 
A behavior tree is made of nodes that each return one of three statuses:
 
| Status | Meaning |
|---|---|
| `SUCCESS` | The node completed successfully, or a condition is true |
| `FAILURE` | The node failed, or a condition is false |
| `RUNNING` | The node is still in progress |
 
Two composite nodes connect leaf nodes into a tree:
 
**Sequence** — like AND. Ticks children left to right, stops on the first
`FAILURE`. Returns `SUCCESS` only if every child succeeds.
 
**Selector / Fallback** — like OR. Ticks children left to right, stops on
the first `SUCCESS`. Returns `FAILURE` only if every child fails.
 
The tree you will build:
 
```
Root  [Selector/Fallback]
├── Reactive  [Sequence]          ← checked FIRST every tick
│   ├── IsObstacleTooClose        ← Condition leaf   (you implement)
│   └── TurnAway                  ← Action leaf      (you implement)
└── Deliberative  [Sequence]      ← only runs when path is clear
    └── DriveForward              ← Action leaf      (pre-implemented)
```
 
**How it works each tick:**
 
The root Selector checks the Reactive branch first.
 
- **Obstacle present:** `IsObstacleTooClose` returns `SUCCESS` → `TurnAway`
  runs and returns `RUNNING` → Reactive Sequence returns `RUNNING` → root
  returns `RUNNING`. `DriveForward` is never ticked. Robot spins.
- **Path clear:** `IsObstacleTooClose` returns `FAILURE` → Reactive Sequence
  fails immediately (never reaches `TurnAway`) → root tries the Deliberative
  branch → `DriveForward` returns `RUNNING`. Robot drives forward.
Notice: no `if/else` code anywhere in the tree assembly. The priority between
reacting and driving emerges purely from the tree structure.
 
---
 
### Setup
 
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```
 
Open Webots and load:
`src/hybrid_assignment/worlds/break_room.wbt`
 
Launch:
```bash
ros2 launch hybrid_assignment assignment.launch.py
```
 
---
 
### What Gets Launched
 
| Node | Purpose |
|---|---|
| `WebotsController` | Bridges Webots ↔ ROS 2. Publishes `/scan`. Subscribes to `/cmd_vel`. |
| `task2_tree` | **Your behavior tree node** — ticks the tree at 10 Hz |
 
---
 
### Topic Map
 
```
Webots
    └── /scan   ← LaserScan  (sensor_msgs/LaserScan)   used by IsObstacleTooClose
 
Your behavior tree node
    └── /cmd_vel  → TwistStamped  (geometry_msgs/TwistStamped)  drives the robot
```
 
---
 
### How DriveForward works (pre-implemented — read this)
 
`DriveForward` is already written for you. It publishes a constant forward
velocity every tick and returns `RUNNING`. That is the entire implementation:
 
```python
def update(self):
    msg = TwistStamped()
    msg.header.stamp    = self.ros_node.get_clock().now().to_msg()
    msg.twist.linear.x  = DRIVE_SPEED
    msg.twist.angular.z = 0.0
    self.publisher.publish(msg)
    return py_trees.common.Status.RUNNING
```
 
It never returns `SUCCESS` or `FAILURE` because it has no goal to complete —
driving forward is an open-ended behavior that runs until the reactive layer
interrupts it.
 
---
 
### Task 1 — Implement the Leaf Behaviours
 
**File:** `hybrid_assignment/task1_behaviours.py`
 
---
 
**TODO 1a — `IsObstacleTooClose.update()`**
 
This is a **Condition** node. It reads the latest LiDAR scan and checks
whether anything in the forward arc is closer than `OBSTACLE_THRESHOLD`.

#### note: you can view the lidar point cloud from View -> Optional Rendering -> Show Lidar Point Cloud
 
- Return `SUCCESS` if an obstacle is detected.
- Return `FAILURE` if the path is clear.
Hints:
- `self.latest_scan` is `None` until the first scan arrives — guard against
  this or the node crashes on the first tick.
- `self.latest_scan.ranges` holds 360 distances, one per degree. Do not
  assume which index points forward — in this simulation index 0 points
  *behind* the robot. Each reading's direction in radians is
  `angle_min + i * angle_increment`; wrap it to −π…π and 0 is straight ahead.
- Readings of `inf` mean no obstacle in that direction — skip them.
- Only check readings whose angle is within ±`FORWARD_ARC_DEG` of straight ahead.
- Check the front-left arc (indices `0 … arc-1`) and the front-right arc
  (indices `total-arc … total-1`) to cover the full forward cone.
- Use the constants `FORWARD_ARC_DEG` and `OBSTACLE_THRESHOLD` at the top
  of the file — do not hard-code numbers.
---
 
**TODO 1b — `TurnAway.update()`**
 
This is an **Action** node. It publishes a spin-in-place command.
 
- Set `msg.twist.linear.x = 0.0` (do not move forward).
- Set `msg.twist.angular.z = TURN_SPEED` (spin left).
- Publish the message.
- Return `RUNNING`.
The node should always return `RUNNING`. The tree structure decides when to
stop turning — when `IsObstacleTooClose` returns `FAILURE` (path is clear),
the Reactive Sequence fails on its first child and `TurnAway` is never even
called.
 
---
 
### Task 2 — Build and Run the Behavior Tree
 
**File:** `hybrid_assignment/task2_tree.py`
 
The `__init__` method of `PatrolTreeNode` is where you build and start the
tree. The steps are broken into TODOs in the skeleton file.
 
**TODO 2a** — Instantiate the three leaf behaviours.
 
**TODO 2b** — Build the reactive branch: a `Sequence` with `obstacle_check`
first, `turn` second.
 
**TODO 2c** — Build the deliberative branch: a `Sequence` with `drive`.
 
**TODO 2d** — Build the root `Selector` (or `Fallback`) with
`reactive_branch` first, `deliberative_branch` second.
 
**TODO 2e** — Wrap in `py_trees.trees.BehaviourTree`, call `setup()`, create
the tick timer.
 
**TODO 2f** — Implement `_tick`: call `self.tree.tick()` and print the tree
state with `py_trees.display.unicode_tree(..., show_status=True)`.
 
---
 
### Testing
 
Once implemented, the robot should:
 
1. Drive forward continuously in open space.
2. Spin in place when any obstacle enters the forward arc.
3. Resume driving the moment the path clears.
The console printout should alternate between these two states:
 
```
Root [*]                          Root [*]
├── Reactive [✕]                  ├── Reactive [*]
│   └── IsObstacleTooClose [✕]    │   ├── IsObstacleTooClose [✓]
└── Deliberative [*]              │   └── TurnAway [*]
    └── DriveForward [*]          └── Deliberative [-]
                                      └── DriveForward [-]
     (driving)                              (turning)
```
 
Common problems and fixes:
 
| Symptom | Likely cause |
|---|---|
| `AttributeError: module 'py_trees.composites' has no attribute 'Fallback'` | Use `Selector` instead — see version note |
| `TypeError: __init__() missing keyword argument 'memory'` | Add `memory=False` to every composite |
| Robot never moves | `IsObstacleTooClose` always returns `SUCCESS` — check the `inf` guard and arc logic |
| Robot drives into walls | `IsObstacleTooClose` always returns `FAILURE` — check that you compute each reading's angle from `angle_min` and `angle_increment` |
| Robot spins when something is behind it | You are treating index 0 as straight ahead — use the angle instead |
| `AttributeError: 'NoneType'` on first tick | Guard against `self.latest_scan is None` at the start of `update()` |
 
---
 
### What real autonomous robots use
 
Your patrol robot implements the two lower layers of the standard three-layer
hybrid architecture:
 
| Layer | What it does | In this assignment |
|---|---|---|
| **Reactive** | Responds to immediate sensor readings | `IsObstacleTooClose` + `TurnAway` |
| **Deliberative (local)** | Executes an ongoing motion goal | `DriveForward` |
| **Strategic (global)** | Plans which areas to visit and when | **Not present** |
 
The strategic layer is what would make the robot actually *patrol* in a
meaningful sense — visiting specific rooms in a planned order, remembering
which areas it has covered, deciding when to recharge. Without it the robot
performs random-walk exploration: it will cover the space eventually, but not
systematically.
 
In a real ROS 2 system this layer would use a map and a coverage planner
(such as nav2's explore_lite package or a custom frontier-based planner) to
select goals. Those goals would be passed to a local planner — replacing your
`DriveForward` — which handles obstacle avoidance far more sophisticatedly
than spinning in place.
 
Your behavior tree is architecturally identical to those production systems.
The difference is only in the complexity of each leaf: a production condition
node might integrate IMU, camera, and LiDAR; a production action node might
invoke a full DWB local planner. The tree structure — and the priority
mechanism it provides for free — is the same.
 
---
 

 
### File Structure
 
```
hybrid_assignment/
├── hybrid_assignment/
│   ├── task1_behaviours.py   ← YOUR WORK: IsObstacleTooClose, TurnAway
│   │                            (DriveForward is pre-implemented — read it)
│   └── task2_tree.py         ← YOUR WORK: assemble and tick the tree
├── launch/
│   └── assignment.launch.py
├── worlds/
│   └── break_room.wbt
├── package.xml
└── setup.py
```
 
---

## Part B: Learning process, use of AI tools, and self-reflection — 4 points

Part B is an equally important part of the assignment as the technical implementation in Part A. The purpose is to reflect on **what you learned, how you worked, and how AI or other tools and resources affected your learning and problem-solving process**.

You are **not assessed based on how much AI you use**. You may use AI extensively, selectively, or not at all. Instead, you are assessed based on the quality of your reflection, your critical evaluation of your working and learning process, and your demonstrated understanding and ownership of the final solution.

### If you did not use AI

Using AI is **not required and does not affect the maximum number of points available**.

If you chose not to use AI, explain why and focus instead on **how you learned and solved the assignment independently**. Describe the resources, tools, documentation, experimentation, debugging, discussions, or other approaches you used to understand the technical concepts and overcome problems.

The same assessment criteria apply: the important thing is to demonstrate and critically reflect on **how you worked, what you learned, and how you evaluated the correctness of your solution**.

In your presentation, address the following four areas:

### 1. What did you learn?

Describe the most important things you learned while completing Part A.

Focus especially on your **technical learning**. For example, in this assignment you might discuss what you learned about sensor data, noise and drift, filtering, sensor fusion, ROS2 topics, or implementing and debugging the provided code.

Do not simply describe what you did. Explain **what you understand now that you did not understand before the assignment**.

**If you used AI:** Explain how AI helped you understand new concepts, code, technologies, or problems and how your understanding developed through this interaction.

**If you did not use AI:** Explain how you developed this understanding. For example, did you study course materials or documentation, search for examples, experiment with the system, inspect code, discuss problems with others, or learn through trial and error?

### 2. How did you solve problems and use AI, tools, and other resources?

Describe **how you approached the assignment and solved the technical problems you encountered**.

**If you used AI**, explain the significant ways in which AI contributed to your work. For each important use, consider:

- what you were trying to accomplish or understand;
- which tool and model you used;
- why you decided to use AI for this particular purpose;
- how you used it; and
- how useful the result was.

You do not need to report every individual prompt. Instead, focus on the **most important ways in which AI influenced your work**.

You must nevertheless transparently identify the AI tools and models you used in Parts A and B, including the model/version and relevant mode or settings when this information is available.

For example:

> I used ChatGPT with GPT-X in thinking mode to understand why integrating a small constant gyroscope bias produces an increasing heading error. I then used it to help interpret the results produced by my implementation.

**If you did not use AI:** Describe the corresponding problem-solving process. What resources or tools did you use instead? How did you find the information you needed? How did you approach debugging or unfamiliar technical concepts? Explain why you chose these approaches and how useful they were.

### 3. Critically evaluate your tools and problem-solving process

Choose at least **one concrete example of a problem, challenge, or learning situation** from the assignment and analyze how you addressed it.

**If you used AI**, this could be a situation where AI:

- significantly helped you;
- produced an incomplete or incorrect answer;
- suggested an approach that needed modification; or
- provided information whose correctness you needed to verify.

Explain what AI suggested, whether the suggestion was useful, **how you determined whether you could trust it**, and how you tested, modified, improved, or rejected the suggestion.

**If you did not use AI:** Select a corresponding technical challenge and explain how you investigated and solved it. For example, describe how you used documentation, course materials, experimentation, debugging, testing, online resources, or discussions with others. Consider whether your initial approach worked, how you evaluated different alternatives, and how you verified that the final solution was correct.

The purpose is to demonstrate that you can **critically evaluate information, tools, and proposed solutions rather than simply accepting them**.

### 4. Reflect on your own learning and working process

Finally, reflect on your overall experience and how you approached learning during the assignment.

Consider questions such as:

- What was the most difficult part of the assignment?
- What did you understand better after completing it?
- Which problem-solving or learning approaches worked particularly well?
- Which approaches did not work well?
- How did you verify that you understood the technical concepts rather than merely producing a working solution?
- If you completed the assignment again, what would you do differently?

**If you used AI:** Also consider whether AI made you more productive or helped you learn, whether it introduced additional problems or confusion, and whether there were situations where solving or studying something without AI was more useful.

**If you did not use AI:** Reflect on how effective your chosen learning and problem-solving strategies were. Consider whether documentation, experimentation, debugging, course materials, discussions, or other resources supported your learning effectively and whether another approach could have made your work or learning more effective.

Your reflection should demonstrate that **you understand and take responsibility for the work you submitted**, regardless of whether or how much AI contributed to the process.

---


## Returning instructions

The assignment must be submitted in video format. Create a video that reports Parts A and B, demonstrating that you have successfully set up the environment, explaining how you learned to use your codebase, and presenting your self-reflection as well as describing how you used AI in the assignment.

In the video, use software such as Microsoft Teams to record your screen while presenting and recording your voice. In the recording, you should demonstrate that your environment is running correctly and show the most essential parts of your codebase. In addition, you should use PowerPoint, Google Slides, or a similar tool to document Part B and present it in the video.

This process will not only teach you how to present your work to others, but it will also help facilitate peer learning and support among students. Although in this first assignment there is only a small amount of technical implementation to report, you will learn the submission procedure that will be used for the rest of the assignments, which will be much more implementation-oriented.

We recommend using Microsoft Teams, as it allows you to record both your screen and voice. The recordings are automatically uploaded to SharePoint, which makes it easy to share your recording later for the peer review assignment. Alternatively, you may use other software such as QuickTime Player or OBS to record your video and then upload the recording to SharePoint.

**Important:** You must ensure that your recording is accessible to others who have the link. So, via Sharepoint user interface in your browser (see below) define the shared settings so that anyone who has the link can access the file for maximum number of days. Finally, you should ensure for example in privacy mode or another browser that the link is truly accessible without login.

![create_sharelink](https://github.com/TEKS4430/Autumn-2026_Assignment_1/blob/main/screenshots/accessrights.png)

<p align="center">
<img src="https://github.com/TEKS4430/Autumn-2026_Assignment_1/blob/main/screenshots/link_settings.png" width=50% height=50%>    
</p>
