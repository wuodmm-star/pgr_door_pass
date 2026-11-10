# pgr_door_pass

**One-go narrow-door passing for differential-drive robots (powered wheelchairs, AMRs).**
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
| key params | `door.x/y/yaw` (map, either side), `door.wall_thickness`, `footprint.xmin/xmax/half_width` (about the drive axle), `pgr.n` (3), `pgr.k` (2), `pgr.pre` (≤0: derived), `v_far`, `v_near`, `w_max` |

The node takes over from the navigation stack for the door manoeuvre: hand over to it ~2–4 m before the door and
give control back once `~/state` reports `done`.

### Gazebo demo

```bash
ros2 launch pgr_door_pass demo.launch.py door_width:=0.75 x:=-3.2 y:=1.0 yaw:=-0.5
```

## Limitations

- Differential drive (rotation centre on the drive axle). A car-like (front-steering) base needs a turning-radius
  aware approach.
- The approach zone in front of the door must be free of other obstacles: the law does not avoid obstacles, it only
  keeps the footprint clear of the door frame.
- Door detection assumes a straight wall with a gap within ~1.3 m of the predicted door centre.
- Simulation results only.

## Credits / 謝辞

- **経路生成形レギュレータ（PGR: Path-Generating Regulator）の考案**：高島 昭彦
  — 高島昭彦「経路生成形レギュレータによる非ホロノミック車両のフィードバック制御」,
  日本機械学会論文集 C編, 70巻689号, pp.120–126, 2004.
- **PGR の発展と研究指導**：花島 直彦 教授（室蘭工業大学）
  — 例：代 軍「The Extended Path-generating Regulator for Control of Nonholonomic Mobile Robots」, 博士論文, 室蘭工業大学, 2015.
- **本ソフトウェア（窄門通過器）の開発**：阿古達目（AGUDAMU）（室蘭工業大学）
- **研究室**：室蘭工業大学 システム制御工学研究室

PGR was proposed by Akihiko Takashima (2004) and developed by the group of Prof. Naohiko Hanajima at the
Muroran Institute of Technology; this door-passing controller was developed by AGUDAMU (阿古達目) in the System
Control Engineering Laboratory, Muroran Institute of Technology.

## License

Apache-2.0

---

## 中文说明

**差速底盘（电动轮椅、AMR）一次通过窄门的控制器**：ROS 2 节点 + 不依赖任何库的 C99 核心。

- **原理**：在门的坐标系里放两口 PGR（经路生成型调节器，Path-Generating Regulator）势能井。第一口在门前（距离 = 车头长 + 半墙厚 + 摆正距离，轮椅为 1.61 m），
  让整个车身在车头到墙之前摆正；第二口在门后。车头朝向始终跟随势能场的流向，被万向轮带偏、打滑或门位置估计跳动时，
  会从当前位置被自动拉回，不需要重新规划。
- **门框由 2D 激光实时检测**（拟合墙线、找缺口、确认缺口后面是通的），在里程计坐标系里累积，控制器用的是
  "车相对门"的位置（厘米级），不依赖地图定位精度。
- **Gazebo 评测**（万向轮为真实自由关节，4 种门宽 × 50 个起点）：一次通过率 97%（0.72 m 门 96%），
  Nav2 的 DWB 为 41%，RPP、Vector Pursuit、Graceful、TEB 为 2–4%（多数在门前判断"要碰撞"而停下放弃）。
- **局限**：只适用于差速底盘；门前接近区内不能有其他障碍物；目前只有仿真结果。

## 致谢

PGR（经路生成型调节器）由高岛昭彦提出（2004），由室兰工业大学花岛直彦教授的团队持续发展；本窄门通过器由阿古达目（AGUDAMU）
在室兰工业大学系统控制工学研究室开发。详见上方 Credits / 謝辞。
