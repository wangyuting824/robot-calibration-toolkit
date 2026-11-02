"""Read motor telemetry through the installed official OpenArm codec.

Only SDK Motor/CanPacketEncoder/CanPacketDecoder objects are used; no OpenArm
controller, device initialization, enable, disable, zero or target API is called.
Motor coordinates are retained; they are not silently relabelled calibrated q.
"""
import select
import socket
import struct
import time
from core.io import InputError, number
from programs.common.check_openarm import inspect_interface, query_ready, active_driver_present


class OpenArmReadOnlyAdapter:
    capabilities = {"read_state": True, "command": False, "watchdog": False, "stop": False}

    def __init__(self, config):
        import openarm_can as sdk
        self.sdk = sdk
        self.rows = config.get("motors", [])
        self.cycles = config.get("cycles", 3)
        self.timeout = number(config.get("reply_timeout_s", .1))
        if not isinstance(self.cycles, int) or isinstance(self.cycles, bool) or not 1 <= self.cycles <= 20 or not 0 < self.timeout <= 1:
            raise InputError("Configure 1..20 bounded cycles and reply_timeout_s in (0,1]")
        if not self.rows or not config.get("mapping_reference"):
            raise InputError("Provide explicit nominal motor mapping and its source")
        keys = [(r["interface"], r["motor_id"]) for r in self.rows]
        if len(set(keys)) != len(keys) or any(isinstance(i, bool) or not isinstance(i, int) or not 1 <= i <= 8 for _, i in keys):
            raise InputError("Specify unique deployed motor IDs 1..8")
        if active_driver_present():
            raise InputError("Existing hardware controller owns the bus; use its verified state source")
        self.sockets = {}
        self.motors = []
        try:
            for interface, _ in keys:
                if interface in self.sockets:
                    continue
                if not query_ready(inspect_interface(interface)):
                    raise InputError(f"{interface}: restore healthy ERROR-ACTIVE CAN-FD 1M/5M before recording")
                sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[interface] = sock
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                sock.bind((interface,))
                sock.setblocking(False)
            for row in self.rows:
                kind = getattr(sdk.MotorType, row["motor_type"])
                self.motors.append(sdk.Motor(kind, row["motor_id"], row["motor_id"] + 0x10))
        except Exception:
            self.close()
            raise

    def samples(self):
        started = time.monotonic()
        for cycle in range(self.cycles):
            for row, motor in zip(self.rows, self.motors):
                interface, motor_id = row["interface"], row["motor_id"]
                if not query_ready(inspect_interface(interface)):
                    raise InputError(f"{interface}: CAN health changed; recording stopped")
                sock = self.sockets[interface]
                # Discard already queued packets. Never reuse one as a fresh reply.
                while select.select([sock], [], [], 0)[0]:
                    sock.recv(72)
                packet = self.sdk.CanPacketEncoder.create_refresh_command(motor)
                payload = bytes(packet.data)
                if packet.send_can_id != 0x7FF or payload != bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0)):
                    raise InputError("Installed SDK returned an unexpected state-request packet")
                if sock.send(struct.pack("=IBBBB64s", packet.send_can_id, 8, 1, 0, 0, payload)) != 72:
                    raise InputError("Incomplete state-request transmission")
                deadline = time.monotonic() + self.timeout
                response = None
                while time.monotonic() < deadline:
                    if not select.select([sock], [], [], max(0, deadline - time.monotonic()))[0]:
                        break
                    data = sock.recv(72)
                    ident, length = struct.unpack_from("=IB", data)
                    if ident == motor_id + 0x10 and length == 8 and len(data) >= 16 and data[8] & 15 == motor_id:
                        response = data[8:16]
                        break
                if response is None:
                    raise InputError(f"No fresh response from {interface} motor {motor_id}")
                measured = self.sdk.CanPacketDecoder.parse_motor_state_data(motor, list(response))
                if not measured.valid:
                    raise InputError("Official SDK rejected the received state")
                yield {"cycle": cycle, "t": time.monotonic() - started, "interface": interface, "motor_id": motor_id,
                    "joint_label_nominal": row["joint_label"], "motor_type": row["motor_type"],
                    "motor_position_rad": number(measured.position), "motor_velocity_rad_s": number(measured.velocity),
                    "sdk_reported_motor_torque_Nm": number(measured.torque), "mos_temperature_C": measured.t_mos,
                    "rotor_temperature_C": measured.t_rotor, "status_nibble_raw": response[0] >> 4, "raw_payload_hex": response.hex(),
                    "joint_mapping_verified": False, "torque_reference_calibrated": False}

    def command(self, target):
        raise InputError("OpenArmReadOnlyAdapter has no motion interface")

    def close(self):
        for sock in getattr(self, "sockets", {}).values():
            sock.close()
        self.sockets = {}
