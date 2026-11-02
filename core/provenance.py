"""Documentation basis and implementation ownership for every workflow."""
from pathlib import Path
from core.io import load

REGISTRY = Path(__file__).with_name("sources.yaml")
if not REGISTRY.exists():
    REGISTRY = Path(__file__).resolve().parents[1] / "docs" / "sources.yaml"


def basis(command):
    registry = load(REGISTRY)
    item = registry["programs"].get(command, {"kind": "project_workflow", "sources": []})
    return {"implementation": "Project-authored unless backend explicitly identifies NVIDIA's installed official tool",
            **item, "references": [{"id": name, **registry["sources"][name]} for name in item.get("sources", [])]}
