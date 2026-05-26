# Running the AAS stack on AirSim (ArduPilot) — integration notes

This documents how the **aerial-autonomy-stack (AAS) aircraft autonomy** was made to
fly on **AirSim + ArduPilot**, what was tried and failed along the way, the fixes that
landed (commit `7de79c5` on `airsim-backend`), how to run it, and what's still open.

> TL;DR: don't let AAS launch AirSim. **Reuse an externally-run, host-networked AirSim +
> ArduPilot-SITL stack** (the `ardupilot-xfs` compose) and run the **AAS aircraft
> container on `--net=host`**, with MAVROS talking to the SITL's MAVLink TCP server.
> Autonomous arm + takeoff is verified end-to-end.

---

## 1. Final working architecture

```
 HOST (one network namespace: --net=host / network_mode: host)
 ┌──────────────────────────────────────────────────────────────────────┐
 │  AirSim (xfs-latest, UE5, GPU)  ── RPC 41451 ──┐                       │
 │        ▲  JSON 9002/9003 (FDM)                 │                       │
 │        │                                       ▼                       │
 │  ardupilot-slim SITL (Copter1) ── MAVLink ─► TCP 5760  + UDP 14550 ────┼─► QGC
 │        │                                       │                       │
 │        │                               (AAS) MAVROS  ◄── tcp://127.0.0.1:5760
 │        │                                       │                       │
 │  airsim-ros2-bridge (Copter1) ── DDS ──► /Copter1/* (domain 1)         │
 │                                                │                       │
 │  AAS aircraft-container (--net=host, ROS_DOMAIN_ID=1):                 │
 │    ardupilot_interface · offboard_control · mission_node · yolo · kiss │
 └──────────────────────────────────────────────────────────────────────┘
```

- **Sim side** is the user's proven `ardupilot-xfs` compose (unchanged): AirSim
  `dhdevspace/auto_mns:xfs-latest`, `dhdevspace/auto_mns:ardupilot-slim` SITL,
  `airsim-ros2-bridge` (vehicles `CopterN`), QGroundControl — all `network_mode: host`,
  localhost addressing, home `42.764970,-115.579139,1130`.
- **AAS side** runs only the aircraft container on `--net=host`. The aircraft autonomy
  is *unchanged from the real-hardware path* except for the airsim wiring below.

Key addresses/ports (instance 0 / Copter1):
| Link | Endpoint |
|---|---|
| AirSim RPC | `41451` |
| AirSim ↔ ArduPilot FDM (JSON) | UDP `9002` (control), `9003` (sensors) |
| ArduPilot MAVLink (for MAVROS) | **TCP `5760`** |
| ArduPilot MAVLink (for QGC) | UDP `14550` |
| per extra drone | all `+10` |

---

## 2. What was tried and why it failed

The path here was not obvious; these dead ends are recorded so they aren't repeated.

1. **AAS launches AirSim on its own docker bridge network** (`tevv-airsim-xfs` image,
   container IPs `10.42.90.x`, in-container `sim_vehicle.py`).
   → **0 UDP datagrams** exchanged. The AirSim build and `ardupilot-slim` use
   **localhost** addressing internally; the ArduPilot JSON FDM link only works when
   AirSim and the SITL share a network namespace. **Host networking is required.**

2. **PX4 path** (`SIM=airsim AUTOPILOT=px4`).
   → PX4 booted its **own Gazebo**. `simulation_airsim.yml.erb` used `PX4_SYS_AUTOSTART=5140/5141`,
   which are the **Gazebo airframes** (`5140_gz_aas_x500`, set `SIM_GZ_EN 1`); PX4 never
   connected to AirSim HIL on `4560`. (Deferred — needs a non-gz `none`/HIL airframe.)

3. **AAS-built ArduCopter 4.6.3** (inside `simulation-image`, `sim_vehicle.py -f airsim-copter`)
   vs the xfs AirSim JSON backend.
   → JSON exchange never established (0 datagrams; AirSim logged
   `Error while receiving rotor control data`). Switched to the **known-good
   `ardupilot-slim` SITL image** instead.

4. **MAVROS on the slim "secondary" MAVLink stream** (`--serial3 udpclient:0.0.0.0:14551`).
   → Sends to `0.0.0.0`, which never reaches a MAVROS bound on `:14551`
   (`connected: false`, all `nan`). The real, usable endpoint is the SITL's
   **MAVLink TCP server on `5760`**. (The worktree `run_ardupilot_sitl.sh` is misleading;
   the shipped image serves `5760`.)

5. **`sim_vehicle.py` overrides `--sim-address`** — it appends its own
   `--sim-address=127.0.0.1` *after* any passed via `-A`, so the AirSim host was ignored.
   (Relevant only for the in-AAS SITL path; the slim image is used instead now.)

6. **Position topics all `nan`** even after MAVROS connected.
   → ArduPilot does **not** stream `GLOBAL_POSITION_INT` / `LOCAL_POSITION_NED` to MAVROS
   until **requested**. Fixed by calling `/mavros/set_stream_rate` once MAVROS is up.

7. **`mission_node` takeoff rejected** (`Goal rejected / Mission Failed`), and
   `target_system_id` stayed `-1`.
   → This AirSim SITL reports `MAV_STATE = CRITICAL (5)` **persistently**, even though it
   arms and flies. `ardupilot_interface` only treats `STANDBY (3)` as "ready", so it
   never latched `target_system`/`mav_type` and rejected takeoff. Fixed by accepting
   `CRITICAL(5)` under `SIM=airsim`.

---

## 3. The fixes (commit `7de79c5`)

- **`tools_and_docs/sim_run.sh`** — new `SIM=airsim AIRSIM_EXTERNAL=true` mode: launches
  **only** the AAS aircraft container(s) on `--net=host` (no AAS sim/ground/bridge/networks),
  with a self-contained cleanup trap. New `FCU_URL` (default `tcp://127.0.0.1:5760`,
  auto-offset `+10` per drone).
- **`aircraft/aircraft.yml.erb`** — `SIM=airsim` MAVROS branch → `fcu_url:=$FCU_URL`, and
  after MAVROS comes up it auto-calls
  `/mavros/set_stream_rate {stream_id:0, message_rate:10, on_off:true}` so position flows.
- **`aircraft/.../autopilot_interface/src/ardupilot_interface.{cpp,hpp}`** — `airsim_mode_`
  (read from the `SIM` env) + `mav_ready()` that accepts `CRITICAL(5)` as well as
  `STANDBY(3)`, used at the FSM-reset, `target_system` latch, and `takeoff_handle_goal` sites.
- **`simulation/simulation_airsim.yml.erb`** — ardupilot `--add-param-file` → base model
  dir (the per-drone clone isn't generated on this path); `--sim-address` passed as a
  native `sim_vehicle.py` option (used only by the in-AAS SITL path, now superseded).

---

## 4. How to run

```sh
# 1) Your sim (host-networked), single drone, windowed
NUM_DRONES=1 ./launch.sh ardupilot-xfs            # in the runtime-stack repo

# 2) AAS aircraft against it (flight-first: perception off)
cd aerial-autonomy-stack/tools_and_docs
SIM=airsim AIRSIM_EXTERNAL=true AUTOPILOT=ardupilot \
  NUM_QUADS=1 NUM_VTOLS=0 CAMERA=false LIDAR=false ./sim_run.sh

# 3) Mission (autonomous arm + takeoff)
docker exec -d aircraft-container-inst0_1 bash -c \
  "source /opt/ros/humble/setup.bash && source /aas/aircraft_ws/install/setup.bash && \
   ros2 run mission mission --conops yalla.yaml --ros-args -r __ns:=/Drone1 -p use_sim_time:=false"
```

`FCU_URL=tcp://127.0.0.1:5760` is the default; override it for a different drone/instance.

### Verify
- `docker exec aircraft-container-inst0_1 bash -lc 'export ROS_DOMAIN_ID=1; source /opt/ros/humble/setup.bash; ros2 topic echo --once /mavros/state | grep connected'` → `connected: true`
- `ardupilot_interface` status prints `target_system_id: 1`, non-nan position.
- The mission arms + climbs (watch `/mavros/local_position/odom` `z`) in the AirSim window.

---

## 5. Status of conops & perception

- **Orbit conops step — FIXED** (commit `685f382`). Root cause: ArduCopter rejects
  `MAV_CMD_DO_SET_ROI_LOCATION (195)` during mission upload (`WaypointPush` →
  `success=false, wp_transfered=2`); `MAV_CMD_DO_SET_ROI (201)` is accepted. The full
  `yalla.yaml` (takeoff → wait → orbit@AUTO → wait → land/RTL) now runs end to end.
  (Not QGC interference and not the SPLINE command — verified by isolation.)
- **LiDAR — DONE** (commit `851dfa3`). On `SIM=airsim`, `kiss_icp` consumes the bridge's
  `/Copter<id>/LidarSensor1/points` directly (DDS-visible on the aircraft domain);
  `/kiss/odometry` publishes at 50 Hz. Run with `LIDAR=true`.
- **Camera — pending the AirSim settings.** No `/Copter*/.../image` topic is published
  because the user's `settings-ardupilot.json` has **no camera**. To enable: add a camera
  block to that settings file (e.g. AAS `settings.ardupilot.json`'s `front_center`,
  320×240, FOV < 90 to keep a pinhole model) and restart AirSim; the bridge will then
  publish `/Copter<id>/<cam>/image`. Then feed `yolo_node` — preferred for the aircraft-only
  external container: have `yolo_node` subscribe that ROS image topic directly (avoids the
  GStreamer/udp:5600 hop and the `airsim_gst_bridge`, which lives in `simulation-image`,
  not `aircraft-image`). Keep `--dfov` consistent with the AirSim camera FOV.

## 6. Deferred

- **PX4 + AirSim HIL**: needs a non-gz `none`/HIL airframe so PX4 connects to AirSim on
  `4560` instead of spawning Gazebo.
