from core.io import result, load
from core.schemas import parameters, validate_parameters


def run(args):
    checked = validate_parameters(parameters(args.parameters), args.require_inertia)
    if args.mechanisms:
        mechanisms = load(args.mechanisms)
        if mechanisms.get("status") != "confirmed" or not mechanisms.get("source"):
            checked["pending"].append("Mechanism definition/source remains unconfirmed")
            checked["ready"] = False
        loops = mechanisms.get("closed_loops", [])
        if loops and mechanisms.get("constraint_implementation_status") != "confirmed":
            checked["pending"].append("Physical closed loops require a validated simulator-specific constraint implementation")
            checked["ready"] = False
    status = "failed" if checked["errors"] else "ready" if checked["ready"] else "pending"
    return result("validate-structure", checked, [args.parameters, args.mechanisms], status)
