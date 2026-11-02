import importlib.util
import sys
from pathlib import Path
from core.io import result, load


def run(args):
    libraries = {name: importlib.util.find_spec(name) is not None for name in ("numpy", "scipy", "yaml", "openpyxl")}
    isaac = importlib.util.find_spec("isaacsim") is not None
    sources = []
    if args.sources:
        config = load(args.sources)
        for item in config.get("files", []):
            path = (Path(args.sources).parent / item["path"]).resolve()
            sources.append({"label": item["label"], "exists": path.is_file(), "path": str(path)})
    ready = all(libraries.values()) and all(item["exists"] for item in sources)
    if args.require_isaac:
        ready &= isaac
    return result("preflight", {"python": sys.version.split()[0], "libraries": libraries,
                  "isaac_importable": isaac, "target_isaac_sim_version": "6.1.0",
                  "isaac_runtime_tested": False, "version_check": "Run examples/run_isaac_smoke.py with the 6.1.0 launcher", "sources": sources},
                  [args.sources], "ready" if ready else "blocked")
