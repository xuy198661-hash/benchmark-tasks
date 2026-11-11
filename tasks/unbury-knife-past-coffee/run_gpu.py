"""Check the task from Git and run explicit, bounded GPU stages by hand.

No command starts/stops a VM, installs packages, or uploads media.
"""

import argparse
from datetime import datetime
import importlib.util
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


HERE = Path(__file__).resolve().parent
EXCLUDED = {"__pycache__", ".pytest_cache", "handoff", "runs", ".DS_Store"}
STEPS = {"calibrate": 100, "smoke": 200, "demo": 1200, "model": 3600}


def task_files():
    return [p for p in sorted(HERE.rglob("*"))
            if p.is_file() and not (set(p.relative_to(HERE).parts) & EXCLUDED) and p.suffix != ".pyc"]


def git_revision():
    """Record the revision being tested without depending on a bundled manifest."""
    try:
        result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=HERE, capture_output=True, text=True)
    except FileNotFoundError:
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def check():
    """CPU-only preflight; never import Isaac Sim or load the entire Arena registry."""
    print(f"Python: {sys.executable}", flush=True)
    if revision := git_revision():
        print(f"Git commit: {revision}", flush=True)
    for name in ("numpy", "pytest", "pxr", "yam_arena"):
        if importlib.util.find_spec(name) is None:
            raise SystemExit(f"Missing {name}; activate the configured yam environment first.")
    from yam_arena.task_file import load
    task = load(HERE / "task.py")
    print(f"Task: {len(task.objects)} objects; source revision {task.sha256[:12]}", flush=True)
    result = subprocess.run([sys.executable, "-m", "pytest", str(HERE), "-q", "-ra", "-p", "no:cacheprovider"])
    if result.returncode:
        raise SystemExit(result.returncode)
    plan = task.demo()
    print("Demo durations (seconds):", {k: round(sum(w.duration for w in v), 2) for k, v in plan.items()})
    print("CPU preflight passed. Simulator contacts and the official mug still require a GPU smoke run.")


def command(mode, cli, run_dir):
    instruction = json.loads((HERE / "task.json").read_text())["instruction"]
    cmd = [cli, "run", "--embodiment", "yam_arena", "--policy", "dream" if mode == "model" else "yam_scripted"]
    for value in (f"task={HERE / 'task.py'}", "object_jitter=0", "control_hz=20", "cam_width=320", "cam_height=176"):
        cmd.extend(["-E", value])
    cmd.extend(["--instruction", instruction, "--scorer", "task_score", "--grader", "none", "--max-steps", str(STEPS[mode]),
                "--store-frames", "--log-dir", str(run_dir / "inspect")])
    return cmd


def run(mode, output_root, dry_run):
    cli = shutil.which("inspect-robots")
    run_dir = output_root.expanduser().resolve() / f"{mode}-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    cmd = command(mode, cli or "inspect-robots", run_dir)
    display_command = f"TASK_DEBUG=1 TASK_CALIBRATION={'1' if mode == 'calibrate' else '0'} " + shlex.join(cmd)
    print(display_command, flush=True)
    if dry_run:
        return
    if cli is None:
        raise SystemExit("inspect-robots not found; source the configured IsaacLab-Arena/yam.env first.")
    gpu = shutil.which("nvidia-smi")
    if gpu is None:
        raise SystemExit("nvidia-smi not found; run this stage in the NVIDIA instance terminal.")
    subprocess.run([gpu, "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"], check=True)
    run_dir.mkdir(parents=True, exist_ok=False)
    snapshot = run_dir / "task-source"
    for p in task_files():
        dest = snapshot / p.relative_to(HERE)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dest)
    (run_dir / "command.txt").write_text(display_command + "\n")
    # Scope debug output to the child process; never read or print credential files.
    import os
    child_env = dict(os.environ, TASK_DEBUG="1", TASK_CALIBRATION="1" if mode == "calibrate" else "0",
                     PYTHONUNBUFFERED="1")
    print(f"Results: {run_dir}", flush=True)
    returncode = 130
    snapshots = []
    try:
        with (run_dir / "console.log").open("w", buffering=1) as log:
            with subprocess.Popen(cmd, env=child_env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, errors="replace", bufsize=1) as proc:
                try:
                    for line in proc.stdout:
                        print(line, end="", flush=True)
                        log.write(line)
                        if "task_state " in line:
                            try:
                                snapshots.append(json.loads(line.split("task_state ", 1)[1]))
                            except json.JSONDecodeError:
                                pass
                    returncode = proc.wait()
                except KeyboardInterrupt:
                    # Ctrl-C reaches the shared terminal process group, including the simulator.
                    if proc.poll() is None:
                        proc.terminate()
                    try:
                        proc.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
    finally:
        with (run_dir / "states.jsonl").open("w") as out:
            for state in snapshots:
                out.write(json.dumps(state) + "\n")
        (run_dir / "run-status.json").write_text(json.dumps({
            "stage": mode, "exit_code": returncode, "max_steps": STEPS[mode],
            "git_commit": git_revision(),
            "last_diagnostic": snapshots[-1] if snapshots else None,
            "note": "The official score is in the Inspect log; diagnostics alone do not prove success.",
        }, indent=2) + "\n")
        print(f"\nSaved logs and exact task sources: {run_dir}", flush=True)
    raise SystemExit(returncode)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="mode", required=True)
    subs.add_parser("check", help="Check files, USDs, scoring and IK without starting Isaac Sim")
    for stage in STEPS:
        run_parser = subs.add_parser(stage, help=f"Run one {stage} stage ({STEPS[stage]} steps maximum)")
        run_parser.add_argument("--output-root", type=Path, default=Path.home() / "yam-eval" / HERE.name)
        run_parser.add_argument("--dry-run", action="store_true", help="Show the command without running anything")
    args = parser.parse_args()
    if args.mode == "check":
        check()
    else:
        run(args.mode, args.output_root, args.dry_run)


if __name__ == "__main__":
    main()
