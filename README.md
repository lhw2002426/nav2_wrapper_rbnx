# nav2_wrapper_rbnx

Robonix package wrapping system-installed Nav2 for the AgileX Ranger Mini stack. Owns `service/navigation/*`. Routes the gRPC contracts (`navigate`, `status`, `cancel`) onto Nav2's `navigate_to_pose` action.

## Capability surface

| Contract                              | Mode | Transport | Source / handler                                      |
| ------------------------------------- | ---- | --------- | ----------------------------------------------------- |
| `robonix/service/navigation/driver`   | rpc  | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle gate      |
| `robonix/service/navigation/navigate` | rpc  | gRPC      | `Navigate(PoseStamped)` → dispatches Nav2 action goal |
| `robonix/service/navigation/status`   | rpc  | gRPC      | `GetNavigationStatus(goal_id)` → status from cache    |
| `robonix/service/navigation/cancel`   | rpc  | gRPC      | `CancelNavigation(goal_id)` → Nav2 cancel_goal_async  |

## Driver-init lifecycle

`start.sh` brings up the atlas bridge — it does NOT spawn nav2. The bridge opens a gRPC server (default port 50235), `RegisterCapability`s, declares only `service/navigation/driver` on atlas, then blocks awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` calls Init, the handler picks the right params YAML (`params_profile: slam | sim | default`, or override with explicit `params_file:`), spawns `ros2 launch nav2_bringup navigation_launch.py params_file:=… use_sim_time:=…`, brings up an `rclpy` node + `ActionClient`, waits for the `navigate_to_pose` action server to advertise (Nav2 lifecycle takes a moment), then declares `navigate / status / cancel` on atlas. If the action server doesn't come up in time, Init returns `state="degraded"` rather than killing nav2 — the operator can investigate.

## Why not vendor nav2?

nav2 is large and the upstream `apt` package builds and ships cleanly on humble. We vendor only the YAML parameter files in `config/` (copied from the robot's working setup so DWB / costmap tunings carry over verbatim). System prereq:

```
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
```

This matches the path the robot was already using.

## Layout

```
nav2_wrapper_rbnx/
├── package_manifest.yaml
├── nav2_wrapper/
│   └── atlas_bridge.py           driver + navigate/status/cancel gRPC
├── scripts/
│   ├── build.sh                  rbnx codegen (no colcon — nav2 is apt)
│   └── start.sh                  source ROS, exec atlas_bridge
└── config/                       VENDORED nav2 params from the robot
    ├── nav2_params.yml
    ├── nav2_params_slam.yml
    └── nav2_params_sim.yml
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "params_profile":  "slam",
  "params_file":     "",
  "use_sim_time":    false,
  "action_wait_s":   45.0,
  "topic_remap":     {}
}
```

`params_profile` selects `config/nav2_params_<profile>.yml`. Set `params_file` to an absolute path (or path relative to the package root) to override entirely — useful when an operator's local tuning lives outside the package.

`topic_remap` is a per-key override of the atlas-resolved bindings (see *Atlas contract dependencies* below). Empty by default — the wrapper picks whatever atlas reports.

## Atlas contract dependencies

The wrapper goes through atlas to find every topic it consumes — it does NOT know which package on a given deploy provides them. At Init time it resolves each contract via `QueryCapabilities` + `ConnectCapability` and passes the resolved topic to `nav2_bringup` as a launch remap.

**Required** (Init returns `state="deferred"` if any are missing on atlas):

| Contract                           | Resolved into nav2 as |
| ---------------------------------- | --------------------- |
| `robonix/service/map/occupancy_grid` | `/map`                |
| `robonix/primitive/chassis/odom`   | `/odom`               |

**Optional** (Init proceeds if absent; the corresponding observation source is just disabled):

| Contract                          | Resolved into nav2 as |
| --------------------------------- | --------------------- |
| `robonix/primitive/lidar/lidar`   | `/scan`               |
| `robonix/primitive/lidar/lidar3d` | `/scanner/cloud`      |

Any binding can be overridden by the deploy manifest's `topic_remap` block — useful if a deploy has multiple providers for the same contract and you need to pin one explicitly. Example:

```yaml
- name: nav2
  url: https://github.com/enkerewpo/nav2_wrapper_rbnx
  config:
    topic_remap:
      map: /robonix/map/occupancy_grid     # bypass atlas resolution
      scan_cloud: /scanner/cloud_filtered  # tap a downstream filter
```

## TF prereqs

Nav2 also needs a healthy `map → odom → base_link → sensor` TF chain. TF is currently a global ROS side-channel rather than an atlas contract — robonix doesn't yet route TF discovery through atlas (open issue). Until it does, ensure whichever stack you deploy publishes the chain. The convention is: the SLAM provider owns `map → odom`, the chassis provider owns `odom → base_link`, and a body-description provider owns `base_link → sensor_*` via `robot_state_publisher`.

If your costmap config (`config/nav2_params_*.yml`) references `/scan` but only `primitive/lidar/lidar3d` (PointCloud2) is on atlas, drop a `pointcloud_to_laserscan` adapter into the launch — or switch the costmap layer to `VoxelLayer` / `ObstacleLayer` with `data_type: PointCloud2`. The upstream `nav2_costmap_2d` supports both.

## License

This package: Apache-2.0 (matches nav2 upstream).
