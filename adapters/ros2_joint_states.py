"""Subscribe to an existing ROS 2 JointState publisher. No command publisher.

Message semantics: ros2/common_interfaces, jazzy/sensor_msgs/msg/JointState.msg.
Publisher timestamps are retained; receipt time is a separate monotonic clock.
The publisher's sign/zero mapping and effort calibration are not verified here.
"""
import time

from core.io import InputError, number


def message_rows(message, sequence, received_t, expected, unavailable_velocity=(), unavailable_effort=()):
    names = list(message.name)
    if not names or len(set(names)) != len(names):
        raise InputError("JointState names must be nonempty and unique")
    if len(message.position) != len(names):
        raise InputError("JointState recording requires one position per joint")
    if set(expected) - set(names):
        raise InputError("JointState is missing configured joints")
    for field in (message.velocity, message.effort):
        if len(field) not in (0, len(names)):
            raise InputError("JointState optional arrays must be empty or match names")
    sec, ns = message.header.stamp.sec, message.header.stamp.nanosec
    if not 0 <= ns < 1_000_000_000:
        raise InputError("Invalid ROS nanosecond timestamp")
    return [{"sequence": sequence, "t": number(received_t), "stamp_sec": sec, "stamp_nanosec": ns,
        "frame_id": message.header.frame_id, "joint": name,
        "q": number(message.position[i]),
        "dq": number(message.velocity[i]) if len(message.velocity) and name not in unavailable_velocity else "",
        "effort": number(message.effort[i]) if len(message.effort) and name not in unavailable_effort else "",
        "dq_published": number(message.velocity[i]) if len(message.velocity) else "",
        "effort_published": number(message.effort[i]) if len(message.effort) else "",
        "joint_mapping_verified": False, "effort_reference_calibrated": False}
        for i, name in enumerate(names)]


class Ros2JointStateAdapter:
    capabilities = {"read_state": True, "command": False, "watchdog": False, "stop": False}

    def __init__(self, config):
        self.count = config.get("messages", 10)
        self.timeout = number(config.get("timeout_s", 10))
        self.expected = config.get("expected_joints", [])
        self.topic = config.get("topic", "/joint_states")
        self.unavailable_velocity = config.get("unavailable_velocity_joints", [])
        self.unavailable_effort = config.get("unavailable_effort_joints", [])
        if isinstance(self.count, bool) or not isinstance(self.count, int) or not 1 <= self.count <= 10000:
            raise InputError("Configure messages in 1..10000")
        if not 0 < self.timeout <= 300 or not isinstance(self.topic, str) or not self.topic.startswith("/"):
            raise InputError("Configure timeout_s in (0,300] and an absolute state topic")
        if not self.expected or len(set(self.expected)) != len(self.expected):
            raise InputError("Configure unique expected_joints to reject unrelated state messages")
        if (set(self.unavailable_velocity) | set(self.unavailable_effort)) - set(self.expected):
            raise InputError("Unavailable fields reference joints outside expected_joints")
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        self.rclpy = rclpy
        self.context = Context()
        self.node = None
        self.executor = None
        self.rclpy.init(args=[], context=self.context)
        try:
            self.node = self.rclpy.create_node("robotcal_joint_state_recorder", context=self.context,
                start_parameter_services=False, enable_rosout=False)
            self.queue = []
            self.node.create_subscription(JointState, self.topic,
                lambda message: self.queue.append((message, time.monotonic())), qos_profile_sensor_data)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except Exception:
            self.close()
            raise

    def samples(self):
        started = time.monotonic()
        previous_stamp = None
        for sequence in range(self.count):
            while not self.queue:
                remaining = self.timeout - (time.monotonic() - started)
                if remaining <= 0:
                    raise InputError(f"Timed out waiting for {self.count} JointState messages on {self.topic}")
                self.executor.spin_once(timeout_sec=min(.1, remaining))
            message, received = self.queue.pop(0)
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp == (0, 0) or (previous_stamp is not None and stamp <= previous_stamp):
                raise InputError("JointState timestamps must be nonzero and strictly increasing")
            rows = message_rows(message, sequence, received - started, self.expected,
                self.unavailable_velocity, self.unavailable_effort)
            previous_stamp = stamp
            yield from rows

    def command(self, target):
        raise InputError("Ros2JointStateAdapter has no command publisher")

    def close(self):
        if self.executor is not None:
            self.executor.shutdown()
            self.executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context.ok():
            self.context.shutdown()
