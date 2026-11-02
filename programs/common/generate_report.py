"""Escape all imported text; report data provenance and qualification without upgrading it."""
from pathlib import Path
from html import escape
import json
from core.io import InputError, load, result, save, csv_rows, number


def run(args):
    folder = Path(args.results)
    records = [load(p) for p in sorted(folder.glob("*.json"))]
    records = [r for r in records if r.get("command") not in ("generate-report", "package-delivery")]
    if not records:
        raise InputError("No workflow JSON results found")
    document = Path(args.destination)
    document.mkdir(parents=True, exist_ok=True)
    parts = ["<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>机器人标定与验证报告</title>",
             "<style>body{font:16px system-ui;max-width:1100px;margin:40px auto;padding:20px}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f6f8;padding:16px}td,th{padding:10px;border:1px solid #bbb}table{border-collapse:collapse}</style>",
             "<h1>机器人标定与验证报告</h1><p>报告保留程序原始状态。synthetic_pass 表示模拟数据算法测试通过，不能证明真机精度；unverified/pending 表示尚未验证。</p>",
             "<table><tr><th>程序</th><th>状态</th><th>运行时间 UTC</th></tr>"]
    for record in records:
        parts.append("<tr>" + "".join("<td>" + escape(str(record.get(k, ""))) + "</td>" for k in ("command", "status", "created_utc")) + "</tr>")
    parts.append("</table>")
    plots = []
    warnings = []
    for index, record in enumerate(records):
        curve = record.get("errors_csv")
        if not curve or not Path(curve).is_file():
            continue
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            warnings.append("Optional matplotlib missing; CSV error curves remain available. Install .[plot] to render plots.")
            break
        rows = csv_rows(curve)
        fig, axis = plt.subplots(figsize=(9, 4), layout="constrained")
        if "position_error_m" in rows[0]:
            plt.close(fig)
            fig, axes = plt.subplots(2, 1, figsize=(9, 6), layout="constrained")
            for ax, key, label in zip(axes, ("position_error_m", "orientation_error_rad"), ("Position error [m]", "Orientation error [rad]")):
                ax.plot([number(r[key]) for r in rows])
                ax.set_ylabel(label)
                ax.set_xlabel("Validation sample index")
                ax.grid(True, alpha=0.3)
        elif "error" in rows[0]:
            # Separate physical units by metric rather than mixing their magnitudes.
            metrics = sorted({r["metric"] for r in rows})
            plt.close(fig)
            fig, axes = plt.subplots(len(metrics), 1, figsize=(9, 3 * len(metrics)), squeeze=False, layout="constrained")
            for metric, ax in zip(metrics, axes[:, 0]):
                for joint in sorted({r["joint"] for r in rows}):
                    sample = [r for r in rows if r["joint"] == joint and r["metric"] == metric]
                    if sample:
                        ax.plot([number(r["t"]) for r in sample], [number(r["error"]) for r in sample], label=joint)
                ax.set_ylabel(metric + " error [SI]")
                ax.set_xlabel("Test-relative time [s]")
                ax.legend()
                ax.grid(True, alpha=0.3)
        else:
            plt.close(fig)
            continue
        name = f"errors_{index}.png"
        fig.savefig(document / name, dpi=160)
        plt.close(fig)
        plots.append(name)
        parts.append("<p>" + escape(record["command"]) + "</p><img style='max-width:100%' src='" + name + "' alt='Validation error curves'>")
    for record in records:
        parts.append("<h2>" + escape(record["command"]) + "</h2><pre>" + escape(json.dumps(record, ensure_ascii=False, indent=2)) + "</pre>")
    parts.append("</html>")
    (document / "report.html").write_text("\n".join(parts), encoding="utf-8")
    save(document / "report.json", {"records": records, "qualification": "No automatic upgrade of fitted/synthetic results to physical validation"})
    return result("generate-report", {"html": str(document / "report.html"), "records": len(records), "plots": plots, "warnings": warnings},
                  list(folder.glob("*.json")), "generated")
