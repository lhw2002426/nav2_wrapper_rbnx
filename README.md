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
  "action_wait_s":   45.0
}
```

`params_profile` selects `config/nav2_params_<profile>.yml`. Set `params_file` to an absolute path (or path relative to the package root) to override entirely — useful when an operator's local tuning lives outside the package.

## TF + topic prereqs

Nav2 wants a healthy `map → odom → base_link` TF chain plus `/scan` (or `/scanner/cloud` projected to scan via `pointcloud_to_laserscan`) for local costmap collision avoidance. In the Ranger Mini deploy these come from:

- `mapping_rbnx` (rtabmap) → `/map` + `map → odom` TF
- `ranger_chassis_rbnx` → `/odom` + `odom → base_link` TF
- `mid360_lidar_rbnx` → `/scanner/cloud`
- soma's `robot_state_publisher` → `base_link → livox_frame / camera_*` TFs

If the costmap config in `nav2_params_slam.yml` references `/scan` directly, you'll need a `pointcloud_to_laserscan` adapter or a costmap layer that consumes PointCloud2 (the upstream `nav2_costmap_2d` ObstacleLayer supports both — adjust `observation_sources` accordingly).

## License

This package: Apache-2.0 (matches nav2 upstream).
