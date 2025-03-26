import argparse
import os
import enum

import numpy as np
import csv
import mujoco as mj
from mj_viewer import MujocoViewer

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="csv file to load", type=str, default=None)
    parser.add_argument("-m", "--mode", help="mode to run the simulation, available modes: PLAYBACK, PD, FF, FF_PD",
                        type=str, default="PLAYBACK")

    args = parser.parse_args()
    if args.file is None:
        # there is not csv file to load, throw an error
        raise ValueError("Please provide a csv file to load")
    else:
        # check if the file exists
        if not os.path.exists("csv/" + args.file + ".csv" if ".csv" not in args.file else "csv/" + args.file):
            raise ValueError("File {} does not exist".format(args.file))
        else:
            print("Loading file: {}".format(args.file))

    # load the csv file
    with open("csv/" + args.file + ".csv" if ".csv" not in args.file else "csv/" + args.file, 'r') as f:
        reader = csv.reader(f)
        data = list(reader)
    motion_data = np.array(data)

    # extract data
    # TODO: Ex.8 - Extract the data from the csv file
    ##### only write your code here #####
    #                                   #
    raise NotImplementedError("Ex.8")
    #####################################

    # load the mujoco model
    model = mj.MjModel.from_xml_path('assets/scene.xml')
    data = mj.MjData(model)

    viewer = MujocoViewer(model, data, hide_menus=False)

    time_counter = 0
    idx = 0

    # reset the initial state
    def reset_state():
        # TODO: Ex.8 - Reset the initial state
        ##### only write your code here #####
        #                                   #
        raise NotImplementedError("Ex.8")
        #####################################

    mj.mj_forward(model, data)

    # simulation rate: 500Hz
    # data rate: 50Hz

    if args.mode == "PLAYBACK":
        # play back the generated motion
        while True:
            if viewer.is_alive:
                # TODO: Ex.8 - Play back the generated motion
                ##### only write your code here #####
                #                                   #
                raise NotImplementedError("Ex.8")
                #####################################
                mj.mj_forward(model, data)
                viewer.render()

            else:
                break
            time_counter += 1

            if idx >= len(t):
                idx = 0  # reset the index, loop the motion
                time_counter = 0
                reset_state()
                print("Looping the motion")

    elif args.mode == "PD":
        # apply a joint PD controller

        while True:
            if viewer.is_alive:
                # TODO: Ex.9 - Apply a joint PD controller
                ##### only write your code here #####
                #                                  #
                raise NotImplementedError("Ex.9")
                #####################################
                mj.mj_step(model, data)
                viewer.render()

            else:
                break
            time_counter += 1

            if idx >= len(t):
                idx = 0  # reset the index, loop the motion
                time_counter = 0
                reset_state()
                print("Looping the motion")

    elif args.mode == "FF":
        # apply the optimized torque directly
        while True:
            if viewer.is_alive:
                # TODO: Ex.10 - Apply the optimized torque directly
                ##### only write your code here #####
                #                                  #
                raise NotImplementedError("Ex.10")
                #####################################
                mj.mj_step(model, data)
                viewer.render()

            else:
                break
            time_counter += 1

            if idx >= len(t):
                idx = 0  # reset the index, loop the motion
                time_counter = 0
                reset_state()
                print("Looping the motion")

    elif args.mode == "FF_PD":
        # apply the optimized torque and a joint PD controller
        k_p = 1.0
        k_d = 0.02
        while True:
            if viewer.is_alive:
                # TODO: Ex.11 - Apply the optimized torque and a joint PD controller
                ##### only write your code here #####
                #                                  #
                raise NotImplementedError("Ex.11")
                #####################################
                mj.mj_step(model, data)
                viewer.render()

            else:
                break
            time_counter += 1

            if idx >= len(t):
                idx = 0  # reset the index, loop the motion
                time_counter = 0
                reset_state()
                print("Looping the motion")
    else:
        raise ValueError("Invalid mode")
