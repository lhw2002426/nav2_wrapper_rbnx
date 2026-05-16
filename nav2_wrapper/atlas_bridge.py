#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""nav2_wrapper_rbnx — atlas bridge (driver-init lifecycle + MCP tools).

Wraps system-installed nav2_bringup. Owns ``robonix/service/navigation/*``.

This bridge plugs into the ``robonix_api.Service`` runtime (same shape as
``services/simple_nav``) so pilot can list / call the navigation tools as
typed MCP endpoints — without that, the nav2 wrapper is reachable only via
raw gRPC and the LLM has no idea how to drive the robot.

Spawn order:
  1. ``start.sh`` launches THIS process via ``python3 -m
     nav2_wrapper.atlas_bridge``.
  2. ``Service.run()`` opens the MCP HTTP server, registers the cap on atlas,
     and blocks awaiting ``Driver(CMD_INIT, config_json)``.
  3. ``rbnx boot`` calls Driver(CMD_INIT). The ``@service.on_init`` handler
     resolves required upstream contracts (map, odom, optionally lidar),
     spawns ``ros2 launch nav2_bringup navigation_launch.py …``, brings up
     a ``rclpy`` node + ``NavigateToPose`` ActionClient, and waits for the
     action server to advertise.
  4. After init, three MCP tools are live for pilot:
       * ``robonix/service/navigation/navigate``
       * ``robonix/service/navigation/status``
       * ``robonix/service/navigation/cancel``

Config (passed via ``Driver(CMD_INIT, config_json)``):
    params_profile   default "slam"  → config/nav2_params_<profile>.yml
                                        (slam | sim | default)
    params_file      unset = derive from params_profile (override w/ abs path)
    use_sim_time     default false
    action_wait_s    default 45.0    — nav2 lifecycle takes a while
    topic_remap      dict, per-key override of atlas-resolved bindings
"""
from __future__ import annotations

import json
import logging
import math
import os
import queue
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

from robonix_api import ATLAS, Service, Ok, Err, Deferred  # noqa: E402

log = logging.getLogger("nav2_wrapper")

nav2 = Service(id=os.environ.get("ROBONIX_CAPABILITY_ID", "nav2"),
               namespace="robonix/service/navigation")

# ── shared state ─────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_pkg_root: Path = Path(__file__).resolve().parent.parent
_nav2_proc: "subprocess.Popen | None" = None
_initialized = False

# ROS2 client state (initialized after nav2 lifecycle is alive)
_ros_node = None
_nav_action_client = None
_nav_action_ready = False
_NavigateToPose = None
_PoseStamped = None
_GoalStatus = None
_nav_queue: "queue.Queue[tuple[str, dict]]" = queue.Queue()
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}
_last_goal_id: str = ""


def _import_ros2() -> None:
    global _NavigateToPose, _PoseStamped, _GoalStatus
    from geometry_msgs.msg import PoseStamped as RosPoseStamped  # type: ignore
    from nav2_msgs.action import NavigateToPose  # type: ignore
    try:
        from action_msgs.msg import GoalStatus  # type: ignore
        _GoalStatus = GoalStatus
    except ImportError:
        _GoalStatus = None
    _PoseStamped = RosPoseStamped
    _NavigateToPose = NavigateToPose


# ── atlas-driven dependency discovery ────────────────────────────────────────
# nav2 needs a few upstream data streams. We DO NOT hardcode which package
# provides them — we ask atlas for each contract and remap the topic into
# nav2 at launch time. This keeps the wrapper coupled to contracts only.
#
# (config_key, contract_id) — config_key matches the launch arg nav2_bringup's
# navigation_launch.py exposes.
_REQUIRED_DEPS: tuple[tuple[str, str], ...] = (
    ("map",   "robonix/service/map/occupancy_grid"),
    ("odom",  "robonix/primitive/chassis/odom"),
)

# Optional deps: if present on atlas, we wire them; otherwise nav2 still
# launches and just won't have that observation source.
_OPTIONAL_DEPS: tuple[tuple[str, str], ...] = (
    ("scan",        "robonix/primitive/lidar/lidar"),
    ("scan_cloud",  "robonix/primitive/lidar/lidar3d"),
)


def _resolve_dep(contract_id: str) -> str | None:
    """Query atlas for a contract over ROS2; return endpoint or None."""
    try:
        caps = ATLAS.find_capability(contract_id=contract_id, transport="ros2")
    except Exception as e:  # noqa: BLE001
        log.warning("query %s failed: %s", contract_id, e)
        return None
    if not caps:
        return None
    try:
        ch = nav2.connect_capability(caps[0], contract_id, "ros2")
    except Exception as e:  # noqa: BLE001
        log.warning("connect %s failed: %s", contract_id, e)
        return None
    ep = ch.endpoint
    try:
        ch.close()
    except Exception:  # noqa: BLE001
        pass
    return ep or None


def _build_remap_args(cfg: dict) -> tuple[list[str], list[str]]:
    overrides = dict(cfg.get("topic_remap", {}) or {})
    remap_args: list[str] = []
    missing: list[str] = []

    for key, contract_id in _REQUIRED_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(contract_id) or ""
        if not ep:
            missing.append(contract_id)
            continue
        remap_args.append(f"{key}:={ep}")
        log.info("resolved %s = %s", contract_id, ep)

    for key, contract_id in _OPTIONAL_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(contract_id) or ""
        if ep:
            remap_args.append(f"{key}:={ep}")
            log.info("resolved (optional) %s = %s", contract_id, ep)
        else:
            log.info("optional dep %s not on atlas — skipping", contract_id)

    return remap_args, missing


# ── nav2 subprocess management ───────────────────────────────────────────────
def _resolve_params_file(cfg: dict) -> str:
    explicit = cfg.get("params_file")
    if explicit:
        p = Path(explicit)
        if not p.is_absolute():
            p = _pkg_root / p
        if not p.is_file():
            raise FileNotFoundError(f"params_file not found: {p}")
        return str(p)
    profile = cfg.get("params_profile", "slam")
    candidates = {
        "slam":    _pkg_root / "config" / "nav2_params_slam.yml",
        "sim":     _pkg_root / "config" / "nav2_params_sim.yml",
        "default": _pkg_root / "config" / "nav2_params.yml",
    }
    p = candidates.get(profile)
    if p is None:
        raise ValueError(
            f"unknown params_profile {profile!r}; options: {list(candidates)}"
        )
    if not p.is_file():
        raise FileNotFoundError(
            f"params file for profile {profile!r} missing: {p}"
        )
    return str(p)


def _spawn_nav2(cfg: dict, remap_args: list[str]) -> None:
    global _nav2_proc
    params_file = _resolve_params_file(cfg)
    use_sim_time = "true" if cfg.get("use_sim_time", False) else "false"
    args = [
        "ros2", "launch", "nav2_bringup", "navigation_launch.py",
        f"use_sim_time:={use_sim_time}",
        f"params_file:={params_file}",
    ]
    args.extend(remap_args)
    log_path = _pkg_root / "rbnx-build" / "data" / "nav2.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning nav2 (params=%s, remaps=%s) → %s",
             params_file, remap_args, log_path)
    _nav2_proc = subprocess.Popen(
        args, stdout=log_fh, stderr=log_fh, start_new_session=True,
    )


def _kill_nav2() -> None:
    p = _nav2_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── ROS2 wiring (started after nav2 is alive) ────────────────────────────────
def _start_ros2_thread() -> None:
    """Spin a rclpy node + ActionClient. Re-entrant: only acts once."""
    def _run():
        global _ros_node, _nav_action_client
        import rclpy  # type: ignore
        from rclpy.executors import MultiThreadedExecutor  # type: ignore
        from rclpy.action import ActionClient  # type: ignore
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: BLE001
            # rclpy may already be initialized by another component; that's OK.
            pass
        node = rclpy.create_node("nav2_wrapper_atlas_bridge")
        _ros_node = node
        _import_ros2()
        _nav_action_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        log.info("rclpy node up; waiting on navigate_to_pose action server")
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            while True:
                try:
                    gid, payload = _nav_queue.get_nowait()
                except queue.Empty:
                    break
                _dispatch_goal(node, gid, payload)
    threading.Thread(target=_run, daemon=True).start()


def _wait_for_action(timeout_s: float) -> bool:
    """Block until ``navigate_to_pose`` is ready (post-Init nav2 lifecycle)."""
    global _nav_action_ready
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _nav_action_client is not None and \
                _nav_action_client.wait_for_server(timeout_sec=0.5):
            _nav_action_ready = True
            return True
        time.sleep(0.5)
    return False


def _make_pose(node, frame_id: str, x: float, y: float, yaw: float):
    g = _PoseStamped()
    g.header.frame_id = frame_id
    g.header.stamp = node.get_clock().now().to_msg()
    g.pose.position.x = float(x)
    g.pose.position.y = float(y)
    g.pose.position.z = 0.0
    g.pose.orientation.z = math.sin(yaw / 2.0)
    g.pose.orientation.w = math.cos(yaw / 2.0)
    return g


def _goal_status_name(status: int) -> str:
    if _GoalStatus is None:
        return str(int(status))
    g = _GoalStatus
    m = {
        int(g.STATUS_UNKNOWN):  "UNKNOWN",
        int(g.STATUS_ACCEPTED): "ACCEPTED",
        int(g.STATUS_EXECUTING): "EXECUTING",
        int(g.STATUS_CANCELING): "CANCELING",
        int(g.STATUS_SUCCEEDED): "SUCCEEDED",
        int(g.STATUS_CANCELED): "CANCELED",
        int(g.STATUS_ABORTED):  "ABORTED",
    }
    return m.get(int(status), str(int(status)))


def _set_state(gid: str, **kw) -> None:
    global _last_goal_id
    with _state_lock:
        _goal_states[gid] = kw
        _last_goal_id = gid


def _goal_response_cb(fut, gid: str):
    try:
        gh = fut.result()
    except Exception as e:  # noqa: BLE001
        _set_state(gid, status="FAILED", accepted=False,
                   terminal=True, error=str(e))
        return
    if not gh.accepted:
        _set_state(gid, status="REJECTED", accepted=False, terminal=True)
        return
    with _state_lock:
        _goal_handles[gid] = gh
        _goal_states[gid] = {"status": "ACCEPTED", "accepted": True,
                             "terminal": False}
    res_fut = gh.get_result_async()
    res_fut.add_done_callback(lambda f: _result_cb(f, gid))


def _result_cb(fut, gid: str):
    try:
        res = fut.result()
        st_name = _goal_status_name(getattr(res, "status", -1))
        with _state_lock:
            _goal_states[gid] = {"status": st_name, "accepted": True,
                                 "terminal": True}
            _goal_handles.pop(gid, None)
    except Exception as e:  # noqa: BLE001
        with _state_lock:
            _goal_states[gid] = {"status": "FAILED", "accepted": True,
                                 "terminal": True, "error": str(e)}
            _goal_handles.pop(gid, None)


def _dispatch_goal(node, gid: str, payload: dict):
    pose = _make_pose(node, payload["frame_id"], payload["x"],
                      payload["y"], payload["yaw"])
    goal_msg = _NavigateToPose.Goal()
    goal_msg.pose = pose
    if _nav_action_client is None or not _nav_action_ready:
        _set_state(gid, status="REJECTED", accepted=False, terminal=True,
                   error="nav action server not ready")
        return
    send_future = _nav_action_client.send_goal_async(goal_msg)
    send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
    _set_state(gid, status="SENT", accepted=False, terminal=False)


def _quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


# ── MCP tools (typed against codegen Request/Response from srv) ──────────────
# The codegen output lives at rbnx-build/codegen/robonix_mcp_types/, exposed
# by start.sh on PYTHONPATH. Mirrors services/simple_nav.
from navigation_mcp import (  # noqa: E402
    Navigate_Request, Navigate_Response,
    GetNavigationStatus_Request, GetNavigationStatus_Response,
    CancelNavigation_Request, CancelNavigation_Response,
)


@nav2.mcp("robonix/service/navigation/navigate")
def navigate(req: Navigate_Request) -> Navigate_Response:
    """Drive the robot to a goal pose using Nav2 ``navigate_to_pose``.

    The request carries a ``geometry_msgs/PoseStamped`` under ``goal``:

        goal.header.frame_id      "map" by default
        goal.pose.position.x|y    map-frame target in meters
        goal.pose.orientation.{z,w} planar-yaw quaternion
                                    (z = sin(yaw/2), w = cos(yaw/2);
                                    leave (z=0,w=1) to skip yaw control)

    Returns ``goal_id`` for polling via the sibling ``status`` / ``cancel``
    contracts. ``status_message`` is a short free-form human-readable string —
    never a JSON envelope.
    """
    if not _initialized:
        return Navigate_Response(
            accepted=False, goal_id="",
            status_message="nav2 not initialized (Driver(CMD_INIT) not run)",
        )
    goal = req.goal
    frame_id = goal.header.frame_id or "map"
    yaw = _quat_to_yaw(goal.pose.orientation.z, goal.pose.orientation.w)
    gid = f"nav-{uuid.uuid4().hex[:8]}"
    _nav_queue.put((gid, {
        "frame_id": frame_id,
        "x": float(goal.pose.position.x),
        "y": float(goal.pose.position.y),
        "yaw": float(yaw),
    }))
    _set_state(gid, status="QUEUED", accepted=False, terminal=False)
    msg = (f"goto ({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})"
           f" yaw={yaw:.2f} frame={frame_id}")
    return Navigate_Response(accepted=True, goal_id=gid, status_message=msg)


@nav2.mcp("robonix/service/navigation/status")
def status(req: GetNavigationStatus_Request) -> GetNavigationStatus_Response:
    """Get current status of a navigation goal.

    Empty ``goal_id`` returns the most recently dispatched goal. ``status``
    is one of ``QUEUED | SENT | ACCEPTED | EXECUTING | CANCELING | SUCCEEDED
    | CANCELED | ABORTED | REJECTED | FAILED | UNKNOWN``. ``terminal`` is
    True once the goal will not change status anymore.
    """
    if not _initialized:
        return GetNavigationStatus_Response(
            known=False, status="nav2 not initialized", terminal=True,
        )
    gid = (req.goal_id or "").strip()
    if not gid:
        gid = _last_goal_id
    with _state_lock:
        st = _goal_states.get(gid)
    if st is None:
        return GetNavigationStatus_Response(
            known=False, status="unknown", terminal=True,
        )
    return GetNavigationStatus_Response(
        known=True,
        status=str(st.get("status", "UNKNOWN")),
        terminal=bool(st.get("terminal", False)),
    )


@nav2.mcp("robonix/service/navigation/cancel")
def cancel(req: CancelNavigation_Request) -> CancelNavigation_Response:
    """Cancel an active navigation goal.

    Empty ``goal_id`` cancels the currently active goal. Idempotent: if
    there is no active goal, returns ``accepted=false`` with an explanatory
    message.
    """
    if not _initialized:
        return CancelNavigation_Response(
            accepted=False, status_message="nav2 not initialized",
        )
    gid = (req.goal_id or "").strip()
    if not gid:
        gid = _last_goal_id
    with _state_lock:
        gh = _goal_handles.get(gid)
    if gh is None:
        return CancelNavigation_Response(
            accepted=False, status_message="no active goal handle",
        )
    try:
        gh.cancel_goal_async()  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return CancelNavigation_Response(
            accepted=False, status_message=f"cancel failed: {e}",
        )
    return CancelNavigation_Response(
        accepted=True, status_message="cancel_requested",
    )


# ── lifecycle ────────────────────────────────────────────────────────────────
@nav2.on_init
def init(cfg):
    """Driver(CMD_INIT). Resolve atlas deps, spawn nav2, wait for action."""
    global _initialized
    with _state_lock:
        if _initialized:
            return Ok()

    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as e:
            return Err(f"bad config_json: {e}")

    action_wait = float(cfg.get("action_wait_s", 45.0))

    remap_args, missing = _build_remap_args(cfg)
    if missing:
        return Deferred(
            f"missing required atlas contracts: {missing} "
            "(awaiting upstream provider)"
        )

    try:
        _spawn_nav2(cfg, remap_args)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn nav2 failed: {e}")

    _start_ros2_thread()

    if not _wait_for_action(action_wait):
        # Don't kill nav2 — degraded state is still useful (operator can
        # investigate). Mark failure but leave the process up.
        return Err(
            f"navigate_to_pose action server did not come up "
            f"within {action_wait:.1f}s"
        )

    with _state_lock:
        _initialized = True
    log.info("init complete: nav2 alive, navigate/status/cancel MCP-exposed")
    return Ok()


def _on_signal(signum, _frame):
    log.info("signal %d — shutting down", signum)
    _kill_nav2()
    raise SystemExit(0)


def main() -> int:
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    try:
        nav2.run()
    finally:
        _kill_nav2()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
