"""Read-only composition for an existing ROS state stream or CSV replay."""
from core.io import InputError, sha256
from core.joint_mapping import JointMapping
from programs.common.capture_data import load_adapter


class MappedJointStateAdapter:
    capabilities = {"read_state": True, "command": False, "stop": False, "watchdog": False}

    def __init__(self, config):
        source = config.get("source", {})
        if source.get("class") not in ("adapters.ros2_joint_states:Ros2JointStateAdapter", "adapters.replay_adapter:ReplayAdapter"):
            raise InputError("MappedJointStateAdapter accepts only the ROS subscriber or CSV replay")
        self.mapping = JointMapping(config["mapping"], config["urdf"], config.get("allow_candidate", False))
        self.replay = source["class"] == "adapters.replay_adapter:ReplayAdapter"
        self.replay_path = source.get("options", {}).get("path") if self.replay else None
        self.source = load_adapter(source)

    def export_metadata(self, meta):
        if self.replay and (not meta.get("sha256") or sha256(self.replay_path) != meta["sha256"]):
            raise InputError("Replay source differs from its dataset manifest")
        converted = self.mapping.metadata(meta)
        if self.replay:
            converted["transport"] = "replay"
        return converted

    def samples(self):
        for row in self.source.samples():
            yield self.mapping.row(row)

    def command(self, target):
        raise InputError("MappedJointStateAdapter has no motion interface")

    def close(self):
        self.source.close()
