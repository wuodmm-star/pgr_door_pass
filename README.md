# pgr_door_pass

**One-go narrow-door passing for wheeled robots (powered wheelchairs, AMRs): differential drive, Ackermann (front
steering) and 4D4T (four-wheel drive, four-wheel independent steering).**
ROS 2 (Humble) node + a dependency-free C99 core (`core/`, libm only).

[中文说明](#中文说明)

A powered wheelchair (1.05 m × 0.66 m) has to go through doors only 0.72–0.90 m wide, i.e. 3–12 cm of clearance
per side, in one go: no stop, no turning on the spot, no reversing. General-purpose local planners tend to stop in
front of such a door ("collision ahead") or scrape the frame. `pgr_door_pass` is a small, dedicated controller for
this one manoeuvre.

## How it works

**PGR (Path-Generating Regulator, 経路生成形レギュレータ) wells in the door frame.**
Around a well pose, with `(X, Y)` the robot position in the well frame, the potential `φ_n = sqrt(X² + n·Y²)` has
the flow direction `θ_r = atan2(−n·Y, −X)` (`X < 0`). Its streamlines `Y = Y0·(X/X0)^n` reach the well tangentially
to the well heading. The robot heading follows the flow:

```
ω = clamp( dθ_r/ds · v  +  k · wrap(θ_r − θ) ,  ±ω_max )       (flow feed-forward + heading feedback)
```

1. **Approach well** in front of the door at `pre = x_front + wall/2 + 5·v_near/k` (front extent + half the wall +
   heading settling distance; 1.61 m for the wheelchair), so the whole body is aligned before its front reaches the wall.
2. **Exit well** beyond the door; slow speed `v_near` near the door.

### Chassis

| chassis | reference point | how the PGR flow is used | command |
|---|---|---|---|
| differential (`config/wheelchair.yaml`) | drive axle | heading follows the flow (above), `n = 3` | `cmd_vel` (v, ω) |
| Ackermann (`config/ackermann.yaml`) | rear axle | same law with the curvature bound `|ω| ≤ v / R_min`; `n = 2` (lower peak curvature of the streamlines); the approach well is moved out so that the car can straighten with a bounded curvature: `pre = x_front + wall/2 + max(5·v_near/k, 2·sqrt(0.2·R_min))` (1.75 m for R_min = 1 m) | (v, ω), or `angular.z` = front steering angle `atan(L·ω/v)` with `ackermann.output_steering_angle: true` |
| 4D4T (`config/4ws.yaml`) | body centre | the **direction of motion** follows the flow; the heading is the one closest to the door normal that keeps the crab angle ≤ `4ws.max_crab_angle` (it turns only when crabbing alone is not enough), so lateral offsets are removed by translating rather than by swinging the body | (vx, vy, ω) |

It is pure state feedback: when the robot is pushed off (caster swing, wheel slip, a jump of the door estimate) it
is pulled back towards a streamline from wherever it is — no re-planning, no optimisation.

**The door is seen in the 2D scan** (RANSAC wall line → gaps → beams through the gap must pass the wall, so the
shadow of an object in front of the wall is not taken for a door) and fused in the odometry frame. The controller
runs on the robot pose *relative to the door* (cm-level), not on the map pose.

## Results (Gazebo 11, physical casters)

Powered wheelchair with **free front casters modelled as real swivel joints** (Gazebo/ODE decides how they swing),
rear-wheel drive (`gazebo_ros_diff_drive`, torque-limited), 270° 2D lidar with 2 cm noise, map/localization error
(6 cm / 2°) + lidar door detection. 4 door widths × the same 50 random starts (2.5–4 m away, ±60° heading),
Gazebo held at 1× real time. **One go** = reached the goal beyond the door with no collision, no reversing, no
turning on the spot and no stop.

| controller | 0.90 m | 0.80 m | 0.75 m | 0.72 m | all | collisions | gave up | CPU / cycle |
|---|---|---|---|---|---|---|---|---|
| **pgr_door_pass** | **98 %** | **100 %** | **94 %** | **96 %** | **97 %** | 5 | 0 | 0.03 ms ¹ |
| Nav2 DWB | 78 % | 54 % | 20 % | 12 % | 41 % | 113 | 0 | 40 ms ² |
| Nav2 RotationShim + RPP | 16 % | 0 % | 0 % | 0 % | 4 % | 47 | 140 | 13 ms ² |
| Vector Pursuit | 16 % | 0 % | 0 % | 0 % | 4 % | 5 | 180 | 13 ms ² |
| Nav2 Graceful | 8 % | 2 % | 0 % | 0 % | 2 % | 50 | 139 | 13 ms ² |
| TEB ³ | 8 % | 0 % | 0 % | – | 3 % | 8 | 52 | 166 ms ² |

Paired exact McNemar test on the same 200 episodes: pgr_door_pass vs DWB 112 : 0 wins, p ≈ 4·10⁻³⁴.

### Decisive: no hesitation, no re-adjusting in front of the door

The robot commits to the door from 2–4 m away and drives through in one continuous motion: it never stops to
re-plan, never turns on the spot to square up and never backs out to try again. Same 200 episodes per controller,
counted before the robot is through the door:

| controller | stop ≥ 0.5 s | turn on the spot | reversing | any of these | gave up |
|---|---|---|---|---|---|
| **pgr_door_pass** | **0.5 %** | **0 %** | **0 %** | **0.5 %** | **0 %** |
| Nav2 DWB | 2.0 % | 11 % | 2.5 % | 14.5 % | 0 % |
| Nav2 RotationShim + RPP | 4.5 % | 27 % | 0.5 % | 32 % | 70 % |
| Vector Pursuit | 5.5 % | 48 % | 4.0 % | 53.5 % | 90 % |
| Nav2 Graceful | 17 % | 33.5 % | 0.5 % | 45.5 % | 69.5 % |
| TEB | 55 % | 85 % | 90 % | 96 % | 38 % |

Why: the PGR flow is defined everywhere, so there is always one forward heading to follow — no search that can come
back empty, no local minimum in front of the jambs, nothing to oscillate between. The approach well is placed
exactly far enough out (`pre`) for the body to be straight before its front reaches the wall.

¹ the control law alone, per 10 Hz cycle. ² CPU time of the whole `controller_server` process per 10 Hz cycle
(includes the local costmap), so the two columns are not the same measurement. ³ 137 of 200 episodes (the 0.72 m
door was not reached). Nav2 controllers: Nav2 Humble 1.1.20 defaults except the rectangular footprint, the
wheelchair speed limits (0.6 m/s, 1 rad/s) and the inflation (0.50 m / scaling 5, so that NavFn's path runs on the
door centre line); global path from NavFn. Most failures are the controller stopping in front of the door
("collision ahead") and the action aborting.

What was also tried and did *not* beat this controller, on the same benchmark: planning the whole approach in
advance with oblique crossings (curvature-bounded candidates, exact footprint checks) and then tracking it —
89 % (91 % with PGR streamlines as the connecting curves, 64 % with Dubins curves). Tracking a plan that was committed
before the door is less robust to the caster disturbances than the PGR feedback.

### Ackermann and 4D4T: same exam, 200 episodes per controller

Same Gazebo exam (4 door widths × 50 starts, 2D lidar with 2 cm noise, localization error + lidar door detection,
1× real time), run with the published C core (`core/`) and, as baselines, the Nav2 Humble controllers the Nav2 docs
recommend for each kinematics. **One go** = goal reached with no collision, no reversing, no turning on the spot,
no stop.

Ackermann (front steering 35°, R_min ≈ 0.87 m; a car cannot turn on the spot, so the starts are 3.8–4.6 m out,
heading ±35°):

| controller | 0.90 m | 0.80 m | 0.75 m | 0.72 m | all | main failure |
|---|---|---|---|---|---|---|
| **pgr_door_pass** | **50/50** | **48/50** | **49/50** | **48/50** | **97.5 %** | 3 collisions |
| Nav2 MPPI (Ackermann model, min_turning_r 1.0) | 34/50 | 2/50 | 0/50 | 0/50 | 18 % | 63 collisions, 45 aborts |
| Nav2 RPP + Smac Hybrid-A* | 21/50 | 3/50 | 0/50 | 0/50 | 12 % | 122 aborts ("collision ahead") |

4D4T (four driven, independently steered wheels; same starts as the differential exam, 2.5–4 m, ±60°):

| controller | 0.90 m | 0.80 m | 0.75 m | 0.72 m | all | main failure |
|---|---|---|---|---|---|---|
| **pgr_door_pass** | **47/50** | **43/50** | **42/50** | **41/50** | **86.5 %** | 27 collisions |
| Nav2 DWB (holonomic, vy ≤ 0.4 m/s) | 44/50 | 19/50 | 3/50 | 2/50 | 34 % | 132 collisions |
| Nav2 MPPI (Omni model) | 15/50 | 0/50 | 0/50 | 0/50 | 7.5 % | 102 timeouts, 58 aborts |

Paired exact McNemar tests on the same 200 episodes: Ackermann pgr_door_pass vs MPPI 159 : 0 (p ≈ 3·10⁻⁴⁸), vs RPP
171 : 0 (p ≈ 7·10⁻⁵²); 4D4T vs DWB 108 : 3 (p ≈ 2·10⁻²⁸), vs MPPI 160 : 2 (p ≈ 5·10⁻⁴⁵). At 0.72 m the Nav2 global
planners found no path (5 cm costmap cells). The 27 4D4T collisions all come from 10 starts that are close to the
wall (2.5–3 m) with a large heading (40–55°) or offset (1.6–1.7 m): a body corner touches the wall next to the door;
a kinematic model of the same base does not reproduce them (200/200), so the steering transients of the wheel
modules are the suspect. Control law CPU time: 0.06 ms per cycle; Nav2 `controller_server` 8–47 ms per cycle
(includes the local costmap). Nav2 settings: `nav2/` of the benchmark (Humble 1.1.20 defaults except footprint,
speed limits, inflation; MPPI from the Nav2 docs example). Gazebo runs where the simulated base blew up (NaN) were
detected and repeated with a fresh model.

Results are from simulation only; the robot has not been tested on hardware yet.

## Build

```bash
cd ~/ros2_ws/src && git clone https://github.com/wuodmm-star/pgr_door_pass.git
cd ~/ros2_ws && colcon build --packages-select pgr_door_pass && colcon test --packages-select pgr_door_pass
```
The C core alone: `cd core && make && ./test_pgr_door`.

## Use

```bash
ros2 launch pgr_door_pass door_pass.launch.py                 # params: config/wheelchair.yaml
ros2 service call /door_pass/start std_srvs/srv/Trigger       # or publish the door and go:
ros2 topic pub --once /door_pass/door geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 4.0, y: 1.2}}}"
```

| | |
|---|---|
| subscribes | `scan` (LaserScan), `odom` (Odometry, for the measured speed), TF `odom→base_link`, `map→odom` |
| publishes | `cmd_vel`, `~/state` (approach / crossing / done), `~/door_pose` (door in the odom frame) |
| services | `~/start`, `~/stop` (std_srvs/Trigger); topic `~/door` sets the door (map frame) and starts |
| key params | `chassis` (`diff` / `ackermann` / `4ws`), `door.x/y/yaw` (map, either side), `door.wall_thickness`, `footprint.xmin/xmax/half_width` (about the drive axle), `pgr.n` (3), `pgr.k` (2), `pgr.pre` (≤0: derived), `v_far`, `v_near`, `w_max` |

The node takes over from the navigation stack for the door manoeuvre: hand over to it ~2–4 m before the door and
give control back once `~/state` reports `done`.

### Gazebo demo

```bash
ros2 launch pgr_door_pass demo.launch.py chassis:=diff door_width:=0.75 x:=-3.2 y:=1.0 yaw:=-0.5
ros2 launch pgr_door_pass demo.launch.py chassis:=ackermann door_width:=0.80 x:=-4.4 y:=1.0 yaw:=-0.3
ros2 launch pgr_door_pass demo.launch.py chassis:=4ws door_width:=0.72 x:=-3.6 y:=-1.3 yaw:=0.7
```

Checked with this node (not the 200-episode benchmark harness): oblique starts, ground-truth footprint vs. jamb
corners, 1.05 m × 0.66 m body for all three (so 12 / 7 / 3 cm is the largest possible clearance at 0.90 / 0.80 /
0.72 m). Through in one go, closest distance to a jamb:

| chassis (Gazebo model) | 0.90 m | 0.80 m | 0.72 m |
|---|---|---|---|
| differential, free swivel casters (`gazebo_ros_diff_drive`) | 3/3, 11.4–11.7 cm | 3/3, 6.4–6.6 cm | 6/6, 2.1–2.8 cm |

No stop, no turning on the spot, no reversing. For the Ackermann and 4D4T bases see the 200-episode results above. On WSL2 the Gazebo-classic real-time throttle can collapse;
pass `pacer_plugin:=<path to a real-time pacer>` or run on native Linux.

## Limitations

- 4D4T: 86.5 % one go; starts close to the wall with a large heading or offset still fail (see above).
- Ackermann: the car cannot turn on the spot, so it needs room: the approach well is ~1.75 m in front of the door
  and the car should take over ~4 m out. Starts closer or more oblique than that are not handled (no reversing).
- The approach zone in front of the door must be free of other obstacles: the law does not avoid obstacles, it only
  keeps the footprint clear of the door frame.
- Door detection assumes a straight wall with a gap within ~1.3 m of the predicted door centre.
- Simulation results only.

## Credits / 謝辞

**経路生成形レギュレータ（PGR: Path-Generating Regulator）**

- 考案：高島 昭彦 ほか「経路生成形レギュレータによる非ホロノミック車両のフィードバック制御」,
  日本機械学会論文集 C編, 70巻689号, pp.120–126, 2004.（独立二輪の移動ロボットに対して提案）
- 四輪車両への拡張：樽海 靖孝, 花島 直彦, 代 軍, 高島 昭彦「経路生成形レギュレータによる四輪車両の走行制御」,
  日本機械学会論文集 C編, 79巻801号, pp.1693–1708, 2013.
- その後の発展（室蘭工業大学 システム制御工学研究室）：直線・円弧経路追従（羅 威 ほか, 2015）、障害物回避
  （大竹 亘 ほか, 日本ロボット学会誌 34(1), 2016）、拡張 PGR（代 軍, 博士論文, 2015）など。

**本ソフトウェア（窄門通過器）**

- 開発：阿古達目（AGUDAMU） — 室蘭工業大学 システム制御工学研究室
- 指導：花島 直彦 教授 — 室蘭工業大学 システム制御工学研究室

PGR was proposed by A. Takashima et al. (2004) and extended to four-wheel vehicles and developed further by the
System Control Engineering Laboratory (Prof. Naohiko Hanajima), Muroran Institute of Technology. This door-passing
controller was developed by AGUDAMU (阿古達目) in the same laboratory under the supervision of Prof. Hanajima.

## License

Apache-2.0

---

## 中文说明

**轮式机器人（电动轮椅、AMR）一次通过窄门的控制器**：ROS 2 节点 + 不依赖任何库的 C99 核心。
支持差速、阿克曼（前轮转向）、4D4T（四驱四转）三种底盘。

- **原理**：在门的坐标系里放两口 PGR（经路生成型调节器，Path-Generating Regulator）势能井。第一口在门前（距离 = 车头长 + 半墙厚 + 摆正距离，轮椅为 1.61 m），
  让整个车身在车头到墙之前摆正；第二口在门后。车头朝向始终跟随势能场的流向，被万向轮带偏、打滑或门位置估计跳动时，
  会从当前位置被自动拉回，不需要重新规划。
- **门框由 2D 激光实时检测**（拟合墙线、找缺口、确认缺口后面是通的），在里程计坐标系里累积，控制器用的是
  "车相对门"的位置（厘米级），不依赖地图定位精度。
- **果断、不徘徊、一次通过**：在门前 2–4 m 就锁定这扇门，一口气开过去，不停车重新规划、不原地转调整姿态、不倒车重试。
  200 局里出现任何犹豫（停顿 ≥0.5 秒、原地转、倒车）的只有 0.5%，放弃 0%；Nav2 各控制器为 14.5–96%，放弃最多 90%。
- **Gazebo 评测**（万向轮为真实自由关节，4 种门宽 × 50 个起点）：一次通过率 97%（0.72 m 门 96%），
  Nav2 的 DWB 为 41%，RPP、Vector Pursuit、Graceful、TEB 为 2–4%（多数在门前判断"要碰撞"而停下放弃）。
- **三种底盘**：差速＝车头跟着 PGR 流线走；阿克曼＝同一律加转弯半径限制（n=2，门前势井放远到约 1.75 m，留出摆正的距离）；
  4D4T＝**行进方向**跟着流线，车头在斜走角不超限的前提下尽量对着门（斜走不够时才转一点），用横移消掉横向偏差，车身不用甩。
  **200 局正式评测**（4 种门宽 × 50 个起点，对照 Nav2 官方推荐的控制器）：阿克曼一次通过 **97.5 %**
  （Nav2 MPPI 18 %、RPP+混合A* 12 %）；4D4T **86.5 %**（Nav2 DWB 34 %、MPPI 7.5 %）；配对检验 p < 10⁻²⁷。
  4D4T 的失败集中在离墙近、朝向很斜的 10 个起点（车角蹭到门旁的墙）。
- **局限**：阿克曼不能原地转，要在门前约 4 m 接手；4D4T 离墙太近又很斜的起点仍会蹭墙；门前接近区内不能有其他
  障碍物；目前只有仿真结果。

## 致谢

PGR（经路生成型调节器）由高岛昭彦等人提出（2004），由室兰工业大学系统控制工学研究室（花岛直彦教授）扩展到四轮车辆并持续发展；
本窄门通过器由阿古达目（AGUDAMU）在花岛直彦教授指导下于该研究室开发。详见上方 Credits / 謝辞。
