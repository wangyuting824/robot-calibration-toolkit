"""Portable operator entry; the ROS worker imports only standard-library core code."""
import sys
import os

if os.environ.get("ROBOTCAL_DISABLE_NETWORK") == "1":
    if len(sys.argv) > 1 and sys.argv[1] == "--ros-worker":
        raise SystemExit("ROS transport is disabled by ROBOTCAL_DISABLE_NETWORK=1")
    from core.offline_guard import install
    install()

if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--ros-worker":
        from core.motion_ros2 import main
        raise SystemExit(main(sys.argv[2]))
    from core.motion_cli import main
    raise SystemExit(main())
