# solara-ros

ROS 2 software for **Solara**, a small autonomous rover that maps a room with a lidar and drives itself to spots you send it to. Written for **ROS 2 Jazzy** on a **Raspberry Pi 5**.

The Pico firmware lives in a separate repo: **solara-firmware**.

## How it works

```
Terminal (SSH) --goal--> Nav2 --cmd_vel--> base_node --serial--> Pico + motors
                          ^                    |
         SLAM Toolbox ----+ map                +-- /odom, odom -> base_link
              ^
Lidar ------/scan------> (SLAM + Nav2 costmaps)
```

| Part | What it does | Who makes it |
|---|---|---|
| `solara_pkg/base_node.py` | `/cmd_vel` to wheel speeds over USB serial; encoder ticks to `/odom` | **us**: the only ROS code |
| `urdf/solara.urdf` | where the lidar is on the robot (`base_link -> laser`) | us, ~15 lines |
| Lidar driver (`rplidar_ros`) | publishes `/scan` | installed |
| SLAM Toolbox | builds the room map; publishes `map -> odom` | installed, `config/slam_*.yaml` |
| Nav2 | plans paths, avoids obstacles, publishes `/cmd_vel` | installed, `config/nav2_params.yaml` |
| `sensors/env_sensor.py` | reads the environmental sensor and prints it | us; plain Python, **not ROS** |
| `foxglove_bridge` | lets Foxglove on a laptop show the map, scan and robot, and send goals | installed, started by the launch file |
| `foxglove_bridge` | streams topics to Foxglove on your laptop so you can see the map | installed, started by the launch file |

**TF tree** (each link has exactly one owner):

```
map -> odom -> base_link -> laser
 SLAM    base_node     URDF
```

## Repo layout

```
solara-ros/
├── solara_pkg/base_node.py     # the one ROS node we write
├── launch/robot.launch.py      # starts everything
├── urdf/solara.urdf
├── config/
│   ├── base_params.yaml        # wheel radius, track width, encoder ticks, serial port
│   ├── nav2_params.yaml        # footprint, speeds, costmaps
│   ├── slam_params.yaml        # mapping mode
│   └── slam_localization_params.yaml
├── maps/                       # saved room maps
├── scripts/
│   ├── goto.sh                 # ./goto.sh 2.0 1.5
│   ├── save_map.sh             # ./save_map.sh room
│   └── 99-solara.rules         # optional: fixed /dev/solara_pico name
└── sensors/
    ├── env_sensor.py           # plain Python, not part of ROS
    └── requirements.txt
```

## Setup on the Pi (once)

```bash
# 1. workspace
mkdir -p ~/solara_ws/src
cd ~/solara_ws/src
git clone <repo-url> solara-ros

# 2. dependencies + build
source /opt/ros/jazzy/setup.bash
cd ~/solara_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

# 3. load ROS + Solara in every new terminal
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/solara_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4. serial port access (log out and back in afterwards)
sudo usermod -aG dialout $USER
```

`--symlink-install` means edits to Python, YAML, launch and URDF files take effect on the next launch. Re-run `colcon build` only after adding new files or changing `setup.py` / `package.xml`.

## Before the first run: measure and fill in

| File | Value | How |
|---|---|---|
| `config/base_params.yaml` | `wheel_radius` | measure the wheel |
| | `ticks_per_rev` | motor/encoder datasheet |
| | `track_width` | start with the measured distance between left and right wheels, then tune with the spin test |
| `urdf/solara.urdf` | lidar `z` | floor to the middle of the lidar head |
| `config/nav2_params.yaml` | `footprint` (2 places) | center of the robot to its outer edges, plus 1-2 cm |
| launch arg `lidar_baud` | baud rate | A1/A2: 115200, A3: 256000, S2: 1000000 |

## Running it

Use `tmux` so the robot keeps running if SSH drops.

**1. Map the room (first time)**

```bash
# window 1
ros2 launch solara_pkg robot.launch.py mode:=mapping

# window 2: drive slowly around the whole room
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# window 2: when finished
~/solara_ws/src/solara-ros/scripts/save_map.sh room
```

Check the map by copying `maps/room.pgm` to your laptop and opening it. Thin, straight walls = good.

**2. Navigate (every time after)**

Put the robot back where mapping started, then:

```bash
# window 1
ros2 launch solara_pkg robot.launch.py mode:=localization

# window 2
~/solara_ws/src/solara-ros/scripts/goto.sh 2.0 1.5
```

To find a spot's coordinates: drive there with teleop, then `ros2 run tf2_ros tf2_echo map base_link`.

**See the robot in Foxglove**

1. Install the Foxglove app on your laptop (foxglove.dev). Your laptop must be on the same hotspot as the Pi.
2. Open connection → **Foxglove WebSocket** → `ws://solara.local:8765`
3. Add a **3D** panel. Turn on `/map`, `/scan`, the TF frames and Nav2's `/plan`.
4. To send a goal by clicking: in the 3D panel's publish tool, choose **Pose**, set its topic to `/goal_pose`, then click a spot and drag to set the heading. Nav2 drives there.

Foxglove is optional: everything still works from the terminal. Start without it using `foxglove:=false`.

**3. Environmental sensor (separate, no ROS)**

```bash
cd ~/solara_ws/src/solara-ros/sensors
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 env_sensor.py
```

## Seeing the map: Foxglove

The launch file starts `foxglove_bridge` on port 8765 (turn it off with `foxglove:=false`).

1. On your laptop, open Foxglove (desktop app or the web app in Chrome). It's free for students with a .edu email, or on the free plan.
2. **Open connection → Foxglove WebSocket →** `ws://solara.local:8765` (laptop on the same hotspot as the Pi).
3. Add a **3D panel** and turn on `/map`, `/scan` and the TF frames. Set the display frame to `map`.
4. **Send goals by clicking:** in the 3D panel's publish tool, choose **Pose**, topic `/goal_pose`, then click and drag on the map. Nav2 drives there.

Over a phone hotspot, stick to the map, scan and TF. Don't stream camera images or point clouds.

## Calibration tests

- **Straight test:** drive 1 m with teleop, compare `/odom` to a tape measure. Off by a constant factor → fix `wheel_radius` or `ticks_per_rev`.
- **Spin test:** rotate 360°. Robot over- or under-turns → adjust `track_width`. Results different every time → wheels are slipping; consider adding an IMU + EKF later (set `publish_tf: false` in `base_params.yaml` when you do).

## Serial protocol (defined in solara-firmware/PROTOCOL.md)

| Direction | Message | Meaning |
|---|---|---|
| Pi → Pico | `V <left> <right>` | target wheel speeds, rad/s, sent at 20 Hz |
| Pico → Pi | `E <left_ticks> <right_ticks>` | running encoder totals, ~50 Hz |
| Pico → Pi | `FW <version>` | firmware version on startup |

The Pico must stop the motors if no `V` arrives for 0.5 s.

## Start-up checklist

```bash
ssh youruser@solara.local
tmux new -s robot                      # or: tmux attach -t robot
ros2 launch solara_pkg robot.launch.py mode:=localization
#   Ctrl-b c  → new tmux window, then e.g. ./scripts/goto.sh 2.0 1.5
#   Ctrl-b d  → detach; robot keeps running
# Laptop: Foxglove → ws://solara.local:8765

# Shut down
#   Ctrl-c in the launch window (wheels stop)
sudo shutdown now                      # wait for the green LED to stop before unplugging
```

## Useful commands

```bash
ros2 topic echo /odom            # is odometry changing?
ros2 topic hz /scan              # is the lidar publishing?
ros2 run tf2_ros tf2_echo map base_link   # where is the robot?
ros2 node list                   # is everything running?
```

`goto.sh` still works when you don't have Foxglove open.

```bash
```
