from importlib import import_module
from pathlib import Path
from core.io import InputError, dataset, load, result, save, sha256, write_csv


def load_adapter(config):
    reference = config.get("class")
    if not isinstance(reference, str) or ":" not in reference:
        raise InputError("Adapter class must be module:Class; only load trusted local adapter code")
    module, cls = reference.split(":", 1)
    try:
        return getattr(import_module(module), cls)(config.get("options", {}))
    except (ImportError, AttributeError) as exc:
        raise InputError(f"Adapter is unavailable: {reference}") from exc


def run(args):
    if args.max_samples <= 0:
        raise InputError("max_samples must be positive")
    cfg = load(args.interface)
    meta = load(args.manifest)
    adapter = load_adapter(cfg)
    if getattr(adapter, "capabilities", {}).get("read_state") is not True:
        raise InputError("Adapter does not support data acquisition")
    try:
        rows = []
        for row in adapter.samples():
            rows.append(row)
            if len(rows) >= args.max_samples:
                break
    finally:
        adapter.close()
    if Path(args.output_data).exists():
        raise InputError("Output dataset already exists; choose a new recording name")
    write_csv(args.output_data, rows)
    meta["sha256"] = sha256(args.output_data)
    if cfg["class"] == "adapters.replay_adapter:ReplayAdapter":
        meta["transport"] = "replay"
    save(args.output_manifest, meta)
    dataset(args.output_data, args.output_manifest)
    return result("capture-data", {"rows": len(rows), "dataset": meta}, [args.interface, args.output_data, args.output_manifest])
