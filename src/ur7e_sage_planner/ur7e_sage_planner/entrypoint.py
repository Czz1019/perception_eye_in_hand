"""Run the copied planner with the selected interpreter and a writable run directory."""
import argparse
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--entry", choices=["servo", "direct", "smoke", "record"], default="servo")
    args, remaining = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + remaining
    default = Path.home() / ".ros/ur7e_sage_planner" / (time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}")
    run_dir = Path(os.environ.get("SAGE_RUN_DIR", str(default))).expanduser().absolute()
    run_dir.mkdir(parents=True, exist_ok=True)
    os.environ["SAGE_RUN_DIR"] = str(run_dir)
    # storm_kit writes control_instance_sage.p in the working directory.
    os.chdir(run_dir)
    print(f"SAGE run directory: {run_dir}", flush=True)
    if args.entry == "record":
        from .record_joint_smoothness import main as run
    elif args.entry == "servo":
        from .reach_static_ur7e_tall_servo_executor import main as run
    else:
        from .reach_static_ur7e_tall import main as run
        if args.entry == "smoke":
            sys.argv.append("--offline-smoke")
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
