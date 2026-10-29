# Construction-Site Hauling Rover: ROS 2 + Gazebo

A simulated differential-drive rover for construction sites. It localises on a
map with AMCL and navigates to goals with Nav2. Navigation accuracy is measured
automatically against simulator ground truth.

**Demo video:** https://drive.google.com/file/d/1yUBrNK178eE0pqdrSsZKPWgJQCNS8syZ/view?usp=drive_link

## What is in here

| Part | What it does |
|---|---|
| `urdf/rover.urdf.xacro` | Robot model. Every dimension, mass and inertia is derived from the formulas below, not guessed. |
| `config/controllers.yaml` | ros2_control differential drive (0.3 m/s, 1.0 rad/s limits). |
| `config/nav2_params.yaml` | Nav2 settings for this robot, merged onto the stock Humble file. |
| `worlds/construction_site.world` | 12 x 10 m site with walls, partitions, pillars and material stacks. |
| `maps/` | Occupancy map, plus `world_to_map.py` that draws it from the world geometry. |
| `scripts/evaluate.py` | Runs N navigation trials and scores them against Gazebo ground truth. |
| `scripts/patrol.py` | Sends the rover through a list of waypoints. |

## Run it

Requires ROS 2 Humble and Gazebo Classic.

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-slam-toolbox

cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
mkdir -p ~/maps && cp src/rover_description/maps/site_map.* ~/maps/

ros2 launch rover_description gazebo.launch.py     # terminal 1
ros2 launch rover_description nav2.launch.py       # terminal 2
python3 src/rover_description/scripts/patrol.py --ros-args -p use_sim_time:=true
```

## Results

Measured over 20 navigation goals chosen at random from 7 free-space points
(fixed seed, goals chained without resets). A goal counts as a verified success
only if Nav2 reports success **and** the robot's true position in Gazebo is
within 25 cm and 20 degrees of the goal.

| Metric | Result |
|---|---|
| Verified success | 19 / 20 (95%, 95% CI 76 to 99%) |
| Mean localisation error | 6.8 cm |
| Mean time per goal | 45.9 s (simulation time) |
| Mean path length per goal | 7.12 m |
| Recovery behaviours per goal | 0.05 |

Reproduce: `python3 scripts/evaluate.py --trials 20 --seed 1 --ros-args -p use_sim_time:=true`

The one failed trial reported success from Nav2, but the true heading ended
outside the 20 degree check. The final error mostly reflects the goal checker's
own tolerance (15 cm, about 14 degrees) plus localisation error.

## Design notes

- **Contact plane:** the wheel and caster bottoms must lie on one plane.
  Wheel joint height = ground + wheel radius, caster height = ground + caster radius.
- **Balance:** with the drive wheels at the centre and a caster at the rear, the
  centre of mass must sit behind the axle. Braking tips the rover forward when
  `a > g * d / H` (d = COM distance behind the axle, H = COM height above ground).
  The chassis is offset 3 cm rearward, giving d = 27 mm and H = 0.14 m, so the
  tipping deceleration is about 1.9 m/s^2 against the 0.5 m/s^2 controller limit.
- **Inertia:** computed from box, cylinder and sphere formulas, not placeholder values.

## Limitations

- Simulation only, with ideal (noise-free) sensors and a flat floor.
- The camera and IMU are visual placeholders; they publish no data yet.
- Odometry accuracy during turns has not yet been calibrated against ground truth.

## Layout

```
rover_description/
  urdf/  config/  launch/  worlds/  maps/  scripts/
```
