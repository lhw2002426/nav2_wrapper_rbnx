# nav2_wrapper_rbnx — goal-based navigation service (Nav2)

Wraps the system-installed ROS2 Nav2 stack and exposes goal-based navigation
to pilot via three MCP tools. Use this capability whenever the user asks the
robot to move, go to a coordinate, navigate to a known pose, stop / cancel
navigation, or check navigation progress.

## Interface (3 MCP tools)

The contracts are typed against the codegen-generated request/response
classes for the `robonix/service/navigation/srv/*` IDL.

### `robonix/service/navigation/navigate`

Send a single Nav2 goal. Returns immediately with a `goal_id`; poll
`status` to track progress.

| field | type | meaning |
|------------------------------|------------------|---------|
| `goal.header.frame_id`       | string           | usually `"map"` |
| `goal.header.stamp.{sec,nanosec}` | int / int   | leave 0 — service stamps the goal at dispatch |
| `goal.pose.position.x`       | float (meters)   | map-frame target X |
| `goal.pose.position.y`       | float (meters)   | map-frame target Y |
| `goal.pose.position.z`       | float            | leave 0.0 |
| `goal.pose.orientation.{x,y,z,w}` | float       | quaternion (planar yaw only) |

Quaternion convention for a planar yaw \(\theta\):

- `x = 0`, `y = 0`
- `z = sin(theta / 2)`
- `w = cos(theta / 2)`

If the user does not specify a final heading, leave the orientation as the
identity quaternion `(x=0, y=0, z=0, w=1)`.

Returns `{accepted, goal_id, status_message}`. `accepted=false` means the
goal was not queued (typically: nav2 not initialized yet). `status_message`
is short free-form text — never JSON.

### `robonix/service/navigation/status`

Poll a previously dispatched goal.

| field | type | meaning |
|--------|--------|---------|
| `goal_id` | string | empty string returns the most recent goal |

Returns `{known, status, terminal}`. `status` is one of:

- non-terminal: `QUEUED`, `SENT`, `ACCEPTED`, `EXECUTING`, `CANCELING`
- terminal: `SUCCEEDED`, `CANCELED`, `ABORTED`, `FAILED`, `REJECTED`,
  `UNKNOWN`

Poll every 1–3 seconds for interactive tasks. Treat the goal as complete
only when `terminal=true` AND `status="SUCCEEDED"`.

### `robonix/service/navigation/cancel`

Cancel an active goal.

| field | type | meaning |
|--------|--------|---------|
| `goal_id` | string | empty string cancels the most recent goal |

Returns `{accepted, status_message}`. Idempotent: cancelling an already-done
or unknown goal returns `accepted=false`.

## Recommended pilot workflow

1. If the user gives explicit map-frame coordinates, build a `PoseStamped`
   with `frame_id="map"` and call `navigate`.
2. If the user names a semantic target (e.g. "go to the chair"), first use
   the scene / semantic-map tools to resolve the target into a safe approach
   pose, then call `navigate`.
3. After dispatch, poll `status` until `terminal=true`. Report success only
   on `SUCCEEDED`.
4. On user interruption / unsafe goal, call `cancel`.

## Safety and preconditions

- Do not invent coordinates for vague natural-language places. Use scene /
  semantic-map, or ask the user for coordinates.
- A healthy TF chain is required: `map -> odom -> base_link -> sensors`.
- A usable occupancy grid must be available from the mapping service.
- Prefer short, local navigation goals over very long moves in cluttered
  environments.
- If Nav2 reports the robot is out of bounds of the costmap, costmaps are
  empty, or TF is broken, do not keep retrying the same goal; report that
  navigation is not ready and ask the operator to recover mapping / TF.

## Dependencies resolved on atlas at init

| key | contract | role |
|-----|----------|------|
| `map`         | `robonix/service/map/occupancy_grid` | required — global costmap |
| `odom`        | `robonix/primitive/chassis/odom`     | required — robot odometry |
| `scan`        | `robonix/primitive/lidar/lidar`      | optional — 2D obstacle source |
| `scan_cloud`  | `robonix/primitive/lidar/lidar3d`    | optional — 3D obstacle source |

If `map` or `odom` is not on atlas at init time the service returns
`Deferred(...)`; rbnx retries once the upstream provider registers. `scan` /
`scan_cloud` are only wired through if the deploy actually publishes them.

## What this service does NOT do

- It does not build maps. Mapping/SLAM provides `/map` and `map -> odom`.
- It does not resolve semantic object names. Scene / semantic-map does that.
- It does not publish robot/sensor TF. Chassis, mapping, and any
  robot-description provider must keep the TF tree healthy.

## Lifecycle

`scripts/start.sh` launches this atlas bridge but does NOT spawn nav2 yet.
nav2 is spawned by `Driver(CMD_INIT, config_json)` once `rbnx boot` calls
the driver — it depends on the resolved atlas topics (`map`, `odom`, …)
which only exist after the upstream services declare on atlas.

Config (passed via `Driver(CMD_INIT, config_json)`):

```json
{
  "params_profile": "slam",
  "params_file":    "",
  "use_sim_time":   false,
  "action_wait_s":  45.0,
  "topic_remap":    {}
}
```

`params_profile` selects `config/nav2_params_<profile>.yml`. Set
`params_file` to an absolute path (or path relative to the package root) to
override the params YAML entirely. `topic_remap` overrides the atlas-resolved
bindings per-key (rare).
