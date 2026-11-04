"""Execute a prevalidated trajectory on an explicit adapter, or replay feedback."""
import time
from core.io import InputError, csv_rows, load, number, result, write_csv
from core.model import Model
from programs.common.capture_data import load_adapter


def run(args):
    interface = load(args.interface)
    if args.target == "replay":
        if interface.get("class") != "adapters.replay_adapter:ReplayAdapter":
            raise InputError("Replay target requires ReplayAdapter")
        adapter = load_adapter(interface)
        try:
            rows = list(adapter.samples())
        finally:
            adapter.close()
        write_csv(args.output_data, rows)
        return result("run-test", {"mode": "replay", "physical_execution": False, "samples": len(rows)}, [args.interface])
    if not args.trajectory or not args.urdf:
        raise InputError("Execution requires trajectory and URDF")
    model = Model(args.urdf)
    trajectory = csv_rows(args.trajectory, ["t", *["q:" + n for n in model.independent]])
    times = [number(r["t"]) for r in trajectory]
    if times[0] != 0 or any(b <= a for a, b in zip(times, times[1:])):
        raise InputError("Trajectory time must start at zero and strictly increase")
    positions = [{n: number(r["q:" + n]) for n in model.independent} for r in trajectory]
    for q in positions:
        model.check_q(q)
    for index in range(1, len(trajectory)):
        dt = times[index] - times[index - 1]
        for name in model.movable:
            before, after = model.positions(positions[index - 1]), model.positions(positions[index])
            limit = model.joints[name].find("limit")
            if limit is None or abs(after[name] - before[name]) / dt > number(limit.get("velocity")) + 1e-9:
                raise InputError(f"Trajectory exceeds {name} velocity limit")
    if args.target == "real":
        if not args.execute:
            raise InputError("Real execution requires explicit --execute; input validation alone never moves hardware")
        procedure = interface.get("operating_procedure") or interface.get("manufacturer_procedure")
        if not isinstance(procedure, str) or not procedure.strip() or not interface.get("commissioning_reference"):
            raise InputError("Real execution requires documented commissioning and an applicable project operating procedure (legacy manufacturer_procedure is also accepted)")
        if not interface.get("limits_confirmed"):
            raise InputError("Real hardware operating limits are not confirmed")
        if interface.get("class") in ("adapters.replay_adapter:ReplayAdapter", "adapters.isaac_adapter:IsaacAdapter"):
            raise InputError("Real target requires a real commissioned robot adapter")
    elif interface.get("class") != "adapters.isaac_adapter:IsaacAdapter":
        raise InputError("Simulation execution requires IsaacAdapter; no numerical substitute is used")
    adapter = load_adapter(interface)
    required = ("command", "read_state", "stop", "watchdog") if args.target == "real" else ("command", "read_state")
    if any(adapter.capabilities.get(k) is not True for k in required):
        adapter.close()
        raise InputError("Adapter lacks required execution capabilities")
    rows = []
    runtime_info = getattr(adapter, "runtime_info", {})
    try:
        first = adapter.read_state()
        tolerance = interface.get("start_tolerance_rad_or_m")
        if tolerance is None:
            raise InputError("Configure start_tolerance_rad_or_m before executing a trajectory")
        for name in model.independent:
            if abs(number(first["q:" + name]) - positions[0][name]) > number(tolerance):
                raise InputError(f"Initial state mismatch: {name}; no automatic repositioning")
        started = time.monotonic()
        for index, (t, q) in enumerate(zip(times, positions)):
            if args.target == "real":
                delay = t - (time.monotonic() - started)
                if delay > 0:
                    time.sleep(delay)
            if args.target == "sim" and index:
                adapter.advance(t - times[index - 1])
            adapter.command(q)
            row = adapter.read_state()
            row.update({"command_t": t, **{"q_command:" + n: v for n, v in q.items()}})
            rows.append(row)
    finally:
        try:
            if getattr(adapter, "capabilities", {}).get("stop"):
                adapter.stop()
        finally:
            adapter.close()
    write_csv(args.output_data, rows)
    return result("run-test", {"mode": args.target, "samples": len(rows), "physical_execution": args.target == "real",
                               "runtime_info": runtime_info}, [args.interface, args.urdf, args.trajectory])
