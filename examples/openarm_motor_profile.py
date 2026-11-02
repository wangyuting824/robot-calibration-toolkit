"""Explicit deployed mapping from OpenArmHW defaults; physical identity unverified."""


def motor_profile(cycles=3):
    kinds = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM4310")
    return {"class": "adapters.openarm_readonly:OpenArmReadOnlyAdapter", "options": {
        "cycles": cycles, "reply_timeout_s": 0.1,
        "mapping_reference": "Deployed OpenArmHW DEFAULT_MOTOR_TYPES/SEND/RECV IDs and startup can0=right, can1=left; physical mapping not verified",
        "motors": [{"interface": bus, "motor_id": i, "motor_type": kind,
                    "joint_label": f"openarm_{side}_joint{i}" if i < 8 else f"openarm_{side}_finger_joint1"}
                   for bus, side in (("can0", "right"), ("can1", "left")) for i, kind in enumerate(kinds, 1)]}}
