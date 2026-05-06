#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""nav2_wrapper_rbnx — atlas bridge (driver-init lifecycle).

Wraps system-installed nav2_bringup. Owns service/navigation/*.

Spawn order:
  1. start.sh launches THIS process — no nav2 spawn yet.
  2. main() starts the gRPC server (Driver + Navigate + Status + Cancel
     servicers), RegisterCapability, declares ONLY service/navigation/driver.
  3. rbnx boot calls Driver(CMD_INIT, config_json).
  4. Init handler: pick params_file from config, spawn `ros2 launch
     nav2_bringup navigation_launch.py …`, wait for the navigate_to_pose
     action server to come up, declare navigate/status/cancel on atlas.

NavigateToPose action client uses the existing /odom + /map + /tf the
rest of the stack provides. Goals are tracked in an internal dict so
status() / cancel() work even after the goal has terminated.

Config (passed via Driver(CMD_INIT, config_json)):
    params_profile   default "slam"     → config/nav2_params_<profile>.yml
                                          (slam | sim | default)
    params_file      unset = derive from params_profile (override w/ abs path)
    use_sim_time     default false
    action_wait_s    default 45.0       — nav2 lifecycle takes a while
"""
from __future__ import annotations

import json
import logging
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid
from concurrent import futures
from pathlib import Path

logging.basicConfig(level=os.environ.get("NAV2_LOG_LEVEL", "INFO"),
                    format="[nav2_wrapper] %(message)s")
log = logging.getLogger("nav2_wrapper")


def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()

import grpc  # noqa: E402
import atlas_pb2 as pb  # noqa: E402
import atlas_pb2_grpc as pb_grpc  # noqa: E402
import lifecycle_pb2  # noqa: E402
import navigation_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402

CMD_INIT = 0
CMD_SHUTDOWN = 1


# ── shared state ─────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_atlas_stub: pb_grpc.AtlasStub | None = None
_cap_id: str = ""
_pkg_root: Path = Path(__file__).resolve().parent.parent
_nav2_proc: subprocess.Popen | None = None
_initialized = False

# ROS2 client state (initialized inside Driver.Init after nav2 is alive)
_ros_node = None
_nav_action_client = None
_nav_action_ready = False
_NavigateToPose = None
_PoseStamped = None
_GoalStatus = None
_nav_queue: "queue.Queue[tuple[str, dict]]" = queue.Queue()
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}


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
# nav2 at launch time. This keeps the wrapper coupled to contracts only;
# whoever publishes them on this deploy is irrelevant.
#
# (config_key, contract_id, default_remap_target) — config_key is the
# string we look up in `cfg["topic_remap"]` so an operator can override
# any individual binding without disabling discovery.
_REQUIRED_DEPS: tuple[tuple[str, str, str], ...] = (
    # robonix/service/map/occupancy_grid → nav2 expects /map for the
    # global costmap's StaticLayer.
    ("map",   "robonix/service/map/occupancy_grid",  "/map"),
    # robonix/primitive/chassis/odom → nav2 + AMCL want /odom.
    ("odom",  "robonix/primitive/chassis/odom",      "/odom"),
)

# Optional deps: if present on atlas, we wire them; if absent, nav2 still
# launches and just won't have that observation source. Useful when the
# deploy has e.g. a 3D lidar but nav2's costmap is configured around 2D
# scan — the operator may legitimately not provide one.
_OPTIONAL_DEPS: tuple[tuple[str, str, str], ...] = (
    # 2D scan for ObstacleLayer (some configs); 3D lidar for VoxelLayer.
    ("scan",        "robonix/primitive/lidar/lidar",   "/scan"),
    ("scan_cloud",  "robonix/primitive/lidar/lidar3d", "/scanner/cloud"),
)


def _resolve_dep(stub, contract_id: str) -> str | None:
    """Query atlas for a contract over ROS2; return endpoint or None."""
    try:
        resp = stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
            contract_id=contract_id,
            transport=pb.TRANSPORT_ROS2,
        ))
    except grpc.RpcError as e:
        log.warning("query %s failed: %s", contract_id, e)
        return None
    for rec in resp.records:
        for iface in rec.interfaces:
            if iface.contract_id != contract_id or iface.transport != pb.TRANSPORT_ROS2:
                continue
            try:
                conn = stub.ConnectCapability(pb.ConnectCapabilityRequest(
                    consumer_id=_cap_id,
                    capability_id=rec.capability_id,
                    contract_id=contract_id,
                    transport=pb.TRANSPORT_ROS2,
                ))
                if conn.endpoint:
                    return conn.endpoint
            except grpc.RpcError as e:
                log.warning("connect %s failed: %s", contract_id, e)
    return None


def _build_remap_args(cfg: dict) -> tuple[list[str], list[str]]:
    """Return (remap_args, missing_required).
    remap_args is a list of `from:=to` strings ready to pass to ros2 launch.
    missing_required is a list of contract_ids that should have been there
    but weren't — caller decides whether to defer / degrade / fail."""
    overrides = dict(cfg.get("topic_remap", {}) or {})
    remap_args: list[str] = []
    missing: list[str] = []

    for key, contract_id, default_target in _REQUIRED_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(_atlas_stub, contract_id) or ""
        if not ep:
            missing.append(contract_id)
            continue
        # ros2 launch syntax: pass remaps via the ros-args mechanism. The
        # nav2_bringup composable nodes pick them up via DeclareLaunchArgument.
        # Cleanest path: rewrite a temp params file with the resolved topic
        # name (the params YAML is where most nav2 nodes look for it).
        remap_args.append(f"{key}:={ep}")
        log.info("resolved %s → %s = %s", contract_id, default_target, ep)

    for key, contract_id, default_target in _OPTIONAL_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(_atlas_stub, contract_id) or ""
        if ep:
            remap_args.append(f"{key}:={ep}")
            log.info("resolved (optional) %s → %s = %s", contract_id, default_target, ep)
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
        raise ValueError(f"unknown params_profile {profile!r}; "
                         f"options: {list(candidates)}")
    if not p.is_file():
        raise FileNotFoundError(f"params file for profile {profile!r} missing: {p}")
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
    # Topic remaps from atlas resolution arrive as launch-arg-shaped
    # `<key>:=<resolved-topic>` pairs. The launch file translates them
    # into ros2 remap ops via `<set_remap>` blocks; for keys the launch
    # doesn't know about we still pass them — no-op if unused. (Future:
    # rewrite the params YAML with substitutions for nodes that read
    # topic names from params rather than via remap.)
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
        global _ros_node, _nav_action_client, _nav_action_ready
        import rclpy  # type: ignore
        from rclpy.executors import MultiThreadedExecutor  # type: ignore
        from rclpy.action import ActionClient  # type: ignore
        rclpy.init(args=None)
        node = rclpy.create_node("nav2_wrapper_atlas_bridge")
        _ros_node = node
        _import_ros2()
        _nav_action_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        log.info("rclpy node up; waiting on navigate_to_pose action server")
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            # Drain goals queued by Navigate gRPC handler.
            while True:
                try:
                    gid, payload = _nav_queue.get_nowait()
                except queue.Empty:
                    break
                _dispatch_goal(node, gid, payload)
    threading.Thread(target=_run, daemon=True).start()


def _wait_for_action(timeout_s: float) -> bool:
    """Block until `navigate_to_pose` is ready (post-Init nav2 lifecycle bring-up)."""
    global _nav_action_ready
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _nav_action_client is not None and _nav_action_client.wait_for_server(timeout_sec=0.5):
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


def _goal_response_cb(fut, gid: str):
    try:
        gh = fut.result()
    except Exception as e:  # noqa: BLE001
        with _state_lock:
            _goal_states[gid] = {"status": "FAILED", "accepted": False,
                                 "terminal": True, "error": str(e)}
        return
    if not gh.accepted:
        with _state_lock:
            _goal_states[gid] = {"status": "REJECTED", "accepted": False,
                                 "terminal": True}
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
    pose = _make_pose(node, payload["frame_id"], payload["x"], payload["y"], payload["yaw"])
    goal_msg = _NavigateToPose.Goal()
    goal_msg.pose = pose
    if _nav_action_client is None or not _nav_action_ready:
        with _state_lock:
            _goal_states[gid] = {"status": "REJECTED", "accepted": False,
                                 "terminal": True,
                                 "error": "nav action server not ready"}
        return
    send_future = _nav_action_client.send_goal_async(goal_msg)
    send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
    with _state_lock:
        _goal_states[gid] = {"status": "SENT", "accepted": False, "terminal": False}


# ── data-interface declaration helpers ───────────────────────────────────────
def _decl_grpc(contract_id: str, port: int, service_name: str, method: str) -> None:
    if _atlas_stub is None:
        return
    _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=_cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_GRPC,
        endpoint=f"127.0.0.1:{port}",
        params=pb.TransportParams(grpc=pb.GrpcParams(
            proto_file="robonix_contracts.proto",
            service_name=service_name,
            method=method,
        )),
    ))


# ── gRPC servicers ───────────────────────────────────────────────────────────
class _NavDriverServicer(contracts_grpc.ServiceNavigationDriverServicer):
    def Driver(self, request, context):
        cmd = int(request.command)
        if cmd == CMD_INIT:
            try:
                cfg = json.loads(request.config_json) if request.config_json else {}
            except json.JSONDecodeError as e:
                return lifecycle_pb2.Driver_Response(
                    ok=False, state="error", error=f"bad config_json: {e}"
                )
            return self._init(cfg)
        if cmd == CMD_SHUTDOWN:
            _kill_nav2()
            return lifecycle_pb2.Driver_Response(ok=True, state="shutdown", error="")
        return lifecycle_pb2.Driver_Response(
            ok=False, state="error", error=f"invalid command {cmd}"
        )

    def _init(self, cfg: dict):
        global _initialized
        with _state_lock:
            if _initialized:
                return lifecycle_pb2.Driver_Response(ok=True, state="ready", error="")

        action_wait = float(cfg.get("action_wait_s", 45.0))

        # Discover upstream deps via atlas. If anything REQUIRED is missing
        # we defer rather than spawn a half-wired nav2; rbnx retries us
        # once the upstream service registers.
        remap_args, missing = _build_remap_args(cfg)
        if missing:
            return lifecycle_pb2.Driver_Response(
                ok=False, state="deferred",
                error=f"missing required atlas contracts: {missing} "
                      f"(awaiting upstream provider)",
            )

        try:
            _spawn_nav2(cfg, remap_args)
        except Exception as e:  # noqa: BLE001
            return lifecycle_pb2.Driver_Response(
                ok=False, state="error", error=f"spawn nav2 failed: {e}"
            )

        # Bring up our rclpy node + ActionClient. nav2 lifecycle takes a
        # while to advertise navigate_to_pose; the ROS2 thread waits.
        _start_ros2_thread()

        if not _wait_for_action(action_wait):
            # Don't kill nav2 — degraded state is still useful (operator
            # can investigate). Mark failure but leave the process up.
            return lifecycle_pb2.Driver_Response(
                ok=False, state="degraded",
                error=f"navigate_to_pose action server did not come up within {action_wait:.1f}s",
            )

        # Declare the data interfaces only after the action server is alive.
        try:
            port = int(os.environ.get("NAV2_DRIVER_PORT", "50235"))
            _decl_grpc("robonix/service/navigation/navigate", port,
                       "ServiceNavigationNavigate", "Navigate")
            _decl_grpc("robonix/service/navigation/status", port,
                       "ServiceNavigationStatus", "GetNavigationStatus")
            _decl_grpc("robonix/service/navigation/cancel", port,
                       "ServiceNavigationCancel", "CancelNavigation")
        except grpc.RpcError as e:
            if e.code() != grpc.StatusCode.ALREADY_EXISTS:
                return lifecycle_pb2.Driver_Response(
                    ok=False, state="error", error=f"declare failed: {e.details()}"
                )

        with _state_lock:
            _initialized = True
        log.info("init complete: nav2 alive, navigate/status/cancel declared")
        return lifecycle_pb2.Driver_Response(ok=True, state="ready", error="")


def _quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


class _NavigateServicer(contracts_grpc.ServiceNavigationNavigateServicer):
    def Navigate(self, request, context):
        if _ros_node is None:
            return navigation_pb2.Navigate_Response(
                accepted=False, status_message="ROS2 not initialized"
            )
        gid = str(uuid.uuid4())
        # request.goal is a geometry_msgs/PoseStamped per the contract IDL.
        goal = request.goal
        frame_id = goal.header.frame_id or "map"
        yaw = _quat_to_yaw(goal.pose.orientation.z, goal.pose.orientation.w)
        _nav_queue.put((gid, {
            "frame_id": frame_id,
            "x": float(goal.pose.position.x),
            "y": float(goal.pose.position.y),
            "yaw": float(yaw),
        }))
        with _state_lock:
            _goal_states[gid] = {"status": "QUEUED", "accepted": False,
                                 "terminal": False}
        return navigation_pb2.Navigate_Response(
            accepted=True,
            status_message=json.dumps({"goal_id": gid, "status": "queued"}),
        )


class _StatusServicer(contracts_grpc.ServiceNavigationStatusServicer):
    def GetNavigationStatus(self, request, context):
        gid = request.goal_id
        with _state_lock:
            st = _goal_states.get(gid)
        if st is None:
            return navigation_pb2.GetNavigationStatus_Response(
                known=False, status="unknown", terminal=True,
            )
        return navigation_pb2.GetNavigationStatus_Response(
            known=True,
            status=st.get("status", "UNKNOWN"),
            terminal=bool(st.get("terminal", False)),
        )


class _CancelServicer(contracts_grpc.ServiceNavigationCancelServicer):
    def CancelNavigation(self, request, context):
        gid = request.goal_id
        with _state_lock:
            gh = _goal_handles.get(gid)
        if gh is None:
            return navigation_pb2.CancelNavigation_Response(
                accepted=False, message="no active goal handle",
            )
        try:
            gh.cancel_goal_async()  # type: ignore[union-attr]
        except Exception as e:  # noqa: BLE001
            return navigation_pb2.CancelNavigation_Response(
                accepted=False, message=f"cancel failed: {e}",
            )
        return navigation_pb2.CancelNavigation_Response(
            accepted=True, message="cancel_requested",
        )


def _start_grpc(port: int) -> None:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    contracts_grpc.add_ServiceNavigationDriverServicer_to_server(
        _NavDriverServicer(), server)
    contracts_grpc.add_ServiceNavigationNavigateServicer_to_server(
        _NavigateServicer(), server)
    contracts_grpc.add_ServiceNavigationStatusServicer_to_server(
        _StatusServicer(), server)
    contracts_grpc.add_ServiceNavigationCancelServicer_to_server(
        _CancelServicer(), server)
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    log.info("Navigation gRPC serving on 0.0.0.0:%d", port)


def _heartbeat_loop() -> None:
    while True:
        time.sleep(15.0)
        if _atlas_stub is None:
            continue
        try:
            _atlas_stub.Heartbeat(pb.HeartbeatRequest(capability_id=_cap_id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)


def _on_signal(signum, _frame):
    log.info("signal %d — shutting down", signum)
    _kill_nav2()
    sys.exit(0)


def main() -> None:
    global _atlas_stub, _cap_id
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    driver_port = int(os.environ.get("NAV2_DRIVER_PORT", "50235"))
    _cap_id = os.environ.get(
        "ROBONIX_CAPABILITY_ID", "com.robonix.service.nav2"
    )

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    _start_grpc(driver_port)

    channel = grpc.insecure_channel(atlas_addr)
    _atlas_stub = pb_grpc.AtlasStub(channel)
    pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
    md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
    try:
        _atlas_stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=_cap_id,
            namespace="robonix/service/navigation",
            capability_md_path=md_path,
        ))
        # Declare ONLY the driver gRPC iface up front. navigate / status /
        # cancel go on atlas after Init succeeds.
        _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
            capability_id=_cap_id,
            contract_id="robonix/service/navigation/driver",
            transport=pb.TRANSPORT_GRPC,
            endpoint=f"127.0.0.1:{driver_port}",
            params=pb.TransportParams(grpc=pb.GrpcParams(
                proto_file="robonix_contracts.proto",
                service_name="ServiceNavigationDriver",
                method="Driver",
            )),
        ))
        log.info("registered cap %s, driver iface on :%d (awaiting INIT)",
                 _cap_id, driver_port)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered (re-deploy); ok", _cap_id)
        else:
            log.warning("atlas registration failed: %s", e)

    threading.Thread(target=_heartbeat_loop, daemon=True).start()
    log.info("ready — awaiting Driver(CMD_INIT)")
    try:
        while True:
            time.sleep(60.0)
    except KeyboardInterrupt:
        pass
    finally:
        _kill_nav2()


if __name__ == "__main__":
    main()
