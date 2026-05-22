# Running aerial-autonomy-stack on AirSim (instead of Gazebo)

This directory holds the **AirSim** simulator backend for AAS: AirSim (Cosys-AirSim)
becomes the physics + sensor engine driving PX4/ArduPilot SITL, and the
[`TEVV-Airsim-ROS2-Bridge`](https://github.com/DinoHub/TEVV-Airsim-ROS2-Bridge)
surfaces AirSim's camera + LiDAR into ROS 2. **The `aircraft/` autonomy is unchanged** —
it connects to SITL exactly as in real-hardware deploy mode (PX4 over XRCE-DDS,
ArduPilot over MAVROS).

> Real-time only. The Gymnasium faster-than-real-time stepping (`gz_step`/`zmq_bridge`)
> is Gazebo `WorldControl`-specific and is **not** part of this path.

```
AirSim (Unreal, Windows)                 WSL2 / Docker (Linux)
  RPC :41451  ◄───────── RPC ──────────── TEVV bridge ── ROS2 ─┐
  PX4 HIL :4560 ◄── MAVLink ──┐                                │
  ArduPilot UDP 9002/9003 ◄───┤  PX4/ArduPilot SITL ── XRCE-DDS:8888 / MAVROS ─► AAS aircraft (UNCHANGED)
                              └──────────── airsim_gst_bridge: Image → H.264/RTP udp:5600 ─► yolo_py
```

Contents:
- `settings.px4.json` — AirSim config, one `PX4Multirotor` (`Drone1`), real-time (`LockStep:false`).
- `settings.ardupilot.json` — AirSim config, one `ArduCopter` (`Drone1`).
- camera shim: `../comms/airsim_gst_bridge/` (built into `build/airsim_gst_bridge`).

---

## Quick start (one command, after rebuilding the sim image)

The orchestration is wired into `tools_and_docs/`. After `sim_build.sh` (so the image has the
camera shim + AirSim launcher), start AirSim first, then:

```bash
SIM=airsim AUTOPILOT=px4 ./tools_and_docs/sim_run.sh        # or AUTOPILOT=ardupilot
```

`sim_run.sh` selects `simulation_airsim.yml.erb`, adds a host route to AirSim
(`--add-host=host.docker.internal:host-gateway`), and launches one TEVV bridge container per
drone. Overrides: `AIRSIM_HOST=<ip>` if AirSim isn't at `host.docker.internal` (e.g. on another
machine — use its LAN IP); `BRIDGE_IMAGE=<tag>` for the bridge image.

The phases below are the **manual / de-risk** path — run them first to validate each piece
(especially the PX4 `none` target and the bridge ↔ external-FC interaction) before relying on
the one-command flow.

> **Platform:** the tested target is **native Linux + NVIDIA GPU**. On native docker, the
> `--add-host` route is what makes the external AirSim reachable from the sim bridge network.

---

## Phase 0 — SITL ↔ AirSim link (do this first)

### 1. Install the AirSim settings
Copy the matching template to AirSim's settings location (Linux: `~/Documents/AirSim/settings.json`;
Windows: `%USERPROFILE%\Documents\AirSim\settings.json`):
- PX4 → `settings.px4.json`
- ArduPilot → `settings.ardupilot.json`

Both expose the RPC API on `41451` bound to `0.0.0.0` so the bridge container can reach it.
If a host firewall is active, allow `41451/tcp`, `4560/tcp` (PX4 HIL), `9002-9003/udp`
(ArduPilot) — on native Linux `ufw` is usually inactive and nothing is needed; on Windows open
them in the Windows firewall.

### 2a. PX4 SITL → AirSim
PX4 SITL must run with the **`none`** simulator target (no Gazebo) so AirSim provides physics
over MAVLink HIL (TCP 4560). Keep AAS's XRCE-DDS env so `aircraft/` sees `/Drone1/fmu/*`:
```bash
# in the SITL container/WSL, NO PX4_GZ_* env
PX4_UXRCE_DDS_NS="Drone1" PX4_UXRCE_DDS_PORT=8888 \
PX4_PARAM_UXRCE_DDS_AG_IP=<MicroXRCEAgent host as signed-int32> \
  make px4_sitl_default none_iris      # or: .../bin/px4 -i 0 with the 'none' airframe
```
- AirSim's `Drone1` (`PX4Multirotor`, `UseTcp:true`, `TcpPort:4560`) connects to this PX4.
- If SITL and AirSim are on different hosts (WSL ↔ Windows), set the vehicle's `LocalHostIp`
  to the **Windows** AirSim IP and PX4's `PX4_SIM_HOSTNAME`/`-h` to the same — confirm against
  your PX4 1.16.2 build, this is the one version-sensitive step.
- `LockStep:false` keeps it real-time.

### 2b. ArduPilot SITL → AirSim
ArduPilot drives; AirSim is the physics backend over UDP (9002/9003):
```bash
sim_vehicle.py -v ArduCopter -f airsim-copter \
  -I 0 --sysid 1 \
  -A "--sim-address=<AIRSIM_WINDOWS_IP> --sim-port-in=9003 --sim-port-out=9002" \
  --add-param-file=/aas/simulation_resources/aircraft_models/iris_with_ardupilot_1/ardupilot-4.6.params \
  --out=udp:<sim_subnet>.90.1:8888           # AAS MAVROS plumbing, unchanged
```
- In `settings.ardupilot.json`, `UdpIp` = ArduPilot SITL host, `LocalHostIp` = AirSim (Windows) host.

### 3. Verify (no sensors yet)
- AAS aircraft container starts; `autopilot_interface` connects.
- PX4: `ros2 topic echo /Drone1/fmu/out/vehicle_status` is live. ArduPilot: `/mavros/state` shows connected.
- Arm + takeoff (via QGC or an AAS mission) → the drone flies **in AirSim**.

---

## Phase 1 — LiDAR via the TEVV bridge

Run the bridge in its own container (Model A), pointed at AirSim, sharing the aircraft's
`ROS_DOMAIN_ID` and network:
```bash
docker run --rm --net=host -e ROS_DOMAIN_ID=1 tevv-airsim-ros2-bridge:humble \
  ros2 launch airsim_ros2_bridge single_vehicle.launch.py \
    vehicle_name:=Drone1 host_ip:=host.docker.internal
```
Remap the bridge's point cloud to what KISS-ICP consumes (`/lidar_points`), e.g. add to the
launch or run a `topic_tools relay` / `--ros-args -r`:
```
/Drone1/lidar/points  →  /lidar_points
```
Verify: `ros2 topic hz /lidar_points`, then KISS-ICP odometry appears.

> ⚠️ **Key caveat — `enableApiControl` vs. PX4/ArduPilot.** The bridge's `multirotor_node`
> calls `enableApiControl(true)` on the vehicle at startup, which can fight an external flight
> controller that already owns it. Camera/LiDAR reads (`simGetImages`, `getLidarData`) do **not**
> need API control. If the bridge disturbs PX4/ArduPilot, either (a) run the bridge with control
> disabled / patch out the `enableApiControl` call for externally-controlled vehicles, or
> (b) bypass the bridge for sensors and pull directly via AirSim RPC (see the `airsim_gst_bridge`
> RPC-pull note below + a small LiDAR RPC publisher). This must be validated against your AirSim
> + bridge build.

---

## Phase 2 — Camera → yolo_py (`udp:5600`)

`yolo_py` (x86_64 sim path) ingests **H.264/RTP on `udp:5600`**. Build and run the shim
(`../comms/airsim_gst_bridge/`), which re-streams the bridge's camera image at the identical
wire format `gz_gst_bridge` produced:
```bash
cd /aas/simulation_resources/comms/airsim_gst_bridge
cmake -B build -S . && cmake --build build         # ROS 2 + gstreamer dev must be sourced/installed
./build/airsim_gst_bridge /Drone1/front_center/image <sim_subnet>.90.1 5600 8
```
- Confirm the actual image topic with `ros2 topic list` (bridge names it from the AirSim camera name).
- **FOV consistency:** AirSim camera `FOV_Degrees` is horizontal; `yolo_py` takes a *diagonal*
  `--dfov`. Keep them consistent (the templates use 320×240; compute the matching diagonal FOV
  and pass it to `yolo_py` so bbox→angle math stays correct — see `yolo_node.py:144-155`).
- If the bridge can't run alongside the external FC (caveat above), switch the shim to pull frames
  directly via AirSim RPC `simGetImages` instead of subscribing to the bridge's ROS 2 image topic.

---

## Gotchas
- **`/clock` owner:** in the AirSim path the bridge self-publishes `/clock` (AirSim time). Do **not**
  also run a `ros_gz_bridge` `/clock` pane. All consumers keep `use_sim_time:=true`.
- **Windows/WSL boundary:** AirSim on Windows; SITL + containers in WSL2/Docker. Bridge →
  `host_ip:=host.docker.internal`. AirSim binds `0.0.0.0`.
- **NumPy on Humble:** if `cv_bridge`/yolo hits `numpy.dtype size changed`, `pip install 'numpy<2.0'`.
- **AAS worlds/wind/gimbal** are Gazebo-specific — use AirSim's Unreal environments instead.

See the full design rationale in the approved plan
(`~/.claude/plans/how-easy-is-it-mighty-goblet.md`).
