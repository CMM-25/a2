import argparse
import time
from datetime import datetime
import os

from draw import animate_traj
from generate_reference import generate_reference
from traj_opt import traj_opt
from export import export_to_csv

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name", help="experiment name", type=str, default=None)
    parser.add_argument(
        "-d",
        "--display",
        help="toggle whether to display animation window",
        action="store_true"
    )
    parser.add_argument("-s", "--save", help="toggle whether to save motion", action="store_true")
    parser.add_argument(
        "-e",
        "--export",
        help="toggle whether to export trajectory to csv",
        action="store_true"
    )

    # parse and post-processing
    args = parser.parse_args()
    if args.name is None:
        args.name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if args.save:
        os.makedirs("videos", exist_ok=True)
    if args.export:
        os.makedirs("csv", exist_ok=True)
    print("Experiment name: {}".format(args.name))

    # generate reference trajectory
    print("Generating reference trajectory...")
    X_ref, U_ref, dt = generate_reference(args.name)
    animate_traj(
        X_ref, U_ref, dt, fname=args.name + "-ref" if args.save else None, display=args.display
    )
    if args.export:
        export_to_csv(X_ref, U_ref, dt, args.name + "-ref")

    # solve trajectory optimization
    print("Solving trajectory optimization...")
    start_time = time.time()
    X_sol, U_sol = traj_opt(X_ref, U_ref, dt)
    print("Optimization took {} minutes".format((time.time() - start_time) / 60.0))
    animate_traj(
        X_sol, U_sol, dt, args.name if args.save else None, display=args.display
    )
    if args.export:
        export_to_csv(X_sol, U_sol, dt, args.name)
