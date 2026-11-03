#!/usr/bin/env python3
"""Project-local entry point; also installed as robotcal."""
import os
if os.environ.get("ROBOTCAL_DISABLE_NETWORK") == "1":
    from core.offline_guard import install
    install()
from core.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
