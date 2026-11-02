"""SocketCAN diagnostics and optional bounded OpenArm state requests.

Protocol basis: Enactic openarm_can CanPacketEncoder::create_refresh_command.
No enable/disable, zeroing, mode, parameter or motion commands are implemented.
"""
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time
from core.io import InputError, number, result, save


def inspect_interface(name):
    proc = subprocess.run(["ip", "-j", "-details", "-statistics", "link", "show", name],
                          check=False, capture_output=True, text=True, timeout=5)
    if proc.returncode:
        raise InputError(f"Cannot inspect CAN interface {name}: {proc.stderr.strip()}")
    return json.loads(proc.stdout)[0]


def query_ready(row, allow_warning=False):
    data = row.get("linkinfo", {}).get("info_data", {})
    allowed = ("ERROR-ACTIVE", "ERROR-WARNING") if allow_warning else ("ERROR-ACTIVE",)
    return ("UP" in row.get("flags", []) and data.get("state") in allowed
            and "FD" in data.get("ctrlmode", []) and data.get("bittiming", {}).get("bitrate") == 1_000_000
            and data.get("data_bittiming", {}).get("bitrate") == 5_000_000)


def state_request(motor_id):
    if isinstance(motor_id, bool) or not isinstance(motor_id, int) or not 1 <= motor_id <= 8:
        raise InputError("This deployed OpenArm profile supports only explicit motor IDs 1..8")
    payload = bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0))
    return struct.pack("=IBBBB64s", 0x7FF, 8, 1, 0, 0, payload)


def active_driver_present():
    for path in Path("/proc").iterdir():
        if path.name.isdigit():
            try:
                executable = (path / "cmdline").read_bytes().split(b"\0")[0].decode(errors="replace")
                if Path(executable).name == "ros2_control_node":
                    return True
            except OSError:
                pass
    return False


def run(args):
    names = args.interfaces
    if not names or len(set(names)) != len(names):
        raise InputError("Configure unique CAN interface names")
    duration = number(args.listen_seconds)
    if not 0 < duration <= 30:
        raise InputError("Diagnostic receive duration must be in (0, 30] seconds")
    before = {name: inspect_interface(name) for name in names}
    warning_probe = getattr(args, "probe_warning", False)
    if warning_probe and not args.query_state:
        raise InputError("--probe-warning requires --query-state")
    blocked = []
    if args.query_state:
        if active_driver_present():
            blocked.append("Existing ros2_control_node owns the bus; subscribe to its verified state source")
        blocked.extend(f"{name}: state query requires UP, ERROR-ACTIVE, CAN-FD 1M/5M; configuration was not changed"
                       for name, row in before.items() if not query_ready(row, warning_probe))
    sockets, frames, sent = {}, [], 0
    try:
        for name in names:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sockets[name] = sock
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            sock.bind((name,))
            sock.setblocking(False)
        def receive(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                available, _, _ = select.select(list(sockets.values()), [], [], max(0, deadline - time.monotonic()))
                for sock in available:
                    data = sock.recv(72)
                    ident, length = struct.unpack_from("=IB", data)
                    payload = data[8:8 + min(length, len(data) - 8)]
                    frames.append({"interface": sock.getsockname()[0], "received_monotonic_s": time.monotonic(),
                        "can_id": ident, "payload_hex": payload.hex(), "length": length})
        receive(duration)
        if args.query_state and not blocked:
            for name, sock in sockets.items():
                for motor_id in range(1, 9):
                    if not query_ready(inspect_interface(name), warning_probe):
                        blocked.append(f"{name}: CAN health changed; remaining requests aborted")
                        break
                    if sock.send(state_request(motor_id)) != 72:
                        raise InputError("Incomplete CAN-FD state request")
                    sent += 1
                    receive(0.1)
    except OSError as exc:
        blocked.append(f"CAN receive/state-query error: {exc}; no further requests sent")
    finally:
        for sock in sockets.values():
            sock.close()
    present = {name: sorted({f["can_id"] - 0x10 for f in frames if f["interface"] == name
               and 0x11 <= f["can_id"] <= 0x18 and f["length"] == 8 and len(bytes.fromhex(f["payload_hex"])) == 8
               and bytes.fromhex(f["payload_hex"])[0] & 0x0F == f["can_id"] - 0x10}) for name in names}
    complete = all(ids == list(range(1, 9)) for ids in present.values())
    if not complete:
        blocked.append("Incomplete motor feedback: no fresh joint positions or calibration success are inferred")
    raw_path = Path(args.raw_frames)
    save(raw_path, {"frames": frames, "data_kind": "hardware_transport_diagnostic", "joint_mapping_verified": False})
    return result("check-openarm", {"before": before, "after": {n: inspect_interface(n) for n in names},
        "query_requested": args.query_state, "state_query_frames_sent": sent, "hardware_control_commands_sent": False,
        "warning_bounded_probe": warning_probe,
        "received_frame_count": len(frames), "responding_motor_ids": present, "blocked_reasons": blocked,
        "raw_frames": str(raw_path), "physical_left_right_mapping_verified": False, "is_real_robot_acceptance": False},
        [raw_path], "ready" if complete and not blocked else "blocked")
