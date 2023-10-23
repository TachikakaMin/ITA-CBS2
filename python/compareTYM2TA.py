#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math
import os
import glob
import json
from copy import deepcopy as dp
import subprocess
import uuid
from faker import Faker

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--TYMPath", help="tym c++ program path")
    parser.add_argument("--CBSTAPath", default=None, help="cbs-ta program path")
    parser.add_argument("--map_dir", help="input file containing map")
    parser.add_argument("--seed", help="input file containing map")
    parser.add_argument("--time", help="input file containing map")
    parser.add_argument("--weight1",default = None, help="weight1", type=float)
    parser.add_argument("--weight2",default = None, help="weight2", type=float)
    args = parser.parse_args()
    args.time = int(args.time)
    f1 = Faker()
    Faker.seed(args.seed)

    map_file_paths = []
    if os.path.isdir(args.map_dir):
        map_file_paths = [f for f in glob.glob(args.map_dir + "/*.yaml")]
    else:
        map_file_paths.append(args.map_dir)
    map_file_paths = sorted(map_file_paths)

    for map_file_path in map_file_paths:
        output_name = "tmp_file_first" + args.seed
        if not (args.weight1 is None):
            run_args = ["./" + args.TYMPath, "-i", map_file_path, "-o", output_name, "-w", str(args.weight1)]
        else:
            run_args = ["./" + args.TYMPath, "-i", map_file_path, "-o", output_name]
        time_out = False
        p = subprocess.Popen(run_args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        try:
            p.wait(timeout=args.time)
        except subprocess.TimeoutExpired:
            # print("Timeout! Terminating the subprocess...")
            time_out = True
            p.terminate()

        if not time_out:
            result = 0
            with open(output_name) as output_file:
                result = yaml.safe_load(output_file)
            try:
                tym_ta_runtime = result["statistics"]['TA_runtime']
            except :
                tym_ta_runtime = 0
            try:
                tym_lowlevel_search_time = result["statistics"]['lowlevel_search_time']
            except :
                tym_lowlevel_search_time = 0
            try:
                tym_first_conflict_time = result["statistics"]['firstconflict_runtime']
            except :
                tym_first_conflict_time = 0
            try:
                tym_new_node_time = result["statistics"]['newnode_runtime']
            except :
                tym_new_node_time = 0

            try:
                tym_focal_score_time = result["statistics"]['focal_score_time']
            except :
                tym_focal_score_time = 0

            try:
                tym_ta_changed_time = result["statistics"]['numTaskAssignmentChanged']
            except :
                tym_ta_changed_time = 0

            tym_time = result["statistics"]['runtime']
            tym_cost = result["statistics"]['cost']
            try:
                tym_ta_times = result["statistics"]['numTaskAssignments']
            except :
                tym_ta_times = 0
            lowLevelNode1 = result["statistics"]['total_lowlevel_node']

        else :
            tym_cost = -1
            tym_time = -1
            tym_ta_runtime = -1
            tym_lowlevel_search_time = -1
            tym_first_conflict_time = -1
            tym_new_node_time = -1
            tym_ta_times = -1
            tym_ta_changed_time = -1
            lowLevelNode1 = -1
            tym_focal_score_time = -1

        if args.CBSTAPath is None:
            test_name = os.path.basename(map_file_path)
            print(test_name,
                  tym_ta_runtime, tym_lowlevel_search_time,
                  tym_new_node_time, tym_first_conflict_time, tym_time, tym_focal_score_time, tym_cost,
                  tym_ta_changed_time, tym_ta_times, lowLevelNode1, flush=True)
            continue

        output_name = "tmp_file_second" + args.seed
        if not (args.weight2 is None):
            run_args = ["./" + args.CBSTAPath, "-i", map_file_path, "-o", output_name, "-w", str(args.weight2)]
        else:
            run_args = ["./" + args.CBSTAPath, "-i", map_file_path, "-o", output_name]

        time_out = False
        p = subprocess.Popen(run_args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        try:
            p.wait(timeout=args.time)
        except subprocess.TimeoutExpired:
            # print("Timeout! Terminating the subprocess...")
            time_out = True
            p.terminate()

        if not time_out:
            with open(output_name) as output_file:
                result = yaml.safe_load(output_file)
            try:
                cbs_ta_runtime = result["statistics"]['TA_runtime']
            except :
                cbs_ta_runtime = 0
            try:
                cbsta_lowlevel_search_time = result["statistics"]['lowlevel_search_time']
            except :
                cbsta_lowlevel_search_time = 0
            try:
                cbsta_first_conflict_time = result["statistics"]['firstconflict_runtime']
            except :
                cbsta_first_conflict_time = 0
            try:
                cbsta_new_node_time = result["statistics"]['newnode_runtime']
            except :
                cbsta_new_node_time = 0

            cbsta_time = result["statistics"]['runtime']
            cbsta_cost = result["statistics"]['cost']

            try:
                cbsta_ta_times = result["statistics"]['numTaskAssignments']
            except :
                cbsta_ta_times = 0
            lowLevelNode2 = result["statistics"]['total_lowlevel_node']
            test_name = os.path.basename(map_file_path)
            if args.weight1:
                if tym_cost != -1 and cbsta_cost*args.weight1 < tym_cost:
                    print(test_name, tym_time, tym_cost, cbsta_time, cbsta_cost, "FALSE", flush=True)
                    continue
            elif tym_cost != -1 and cbsta_cost != tym_cost:
                print(test_name, tym_time, tym_cost, cbsta_time, cbsta_cost, "FALSE", flush=True)
                continue

            print(test_name,
                  tym_ta_runtime, tym_lowlevel_search_time,
                  tym_new_node_time, tym_first_conflict_time, tym_time,
                  cbs_ta_runtime, cbsta_lowlevel_search_time,
                  cbsta_new_node_time, cbsta_first_conflict_time, cbsta_time,
                  cbsta_cost, tym_ta_times, cbsta_ta_times, lowLevelNode1, lowLevelNode2, flush=True)
        else :
            test_name = os.path.basename(map_file_path)
            print(test_name,
                  tym_ta_runtime, tym_lowlevel_search_time,
                  tym_new_node_time, tym_first_conflict_time, tym_time,
                  -1, -1,
                  -1, -1, -1,
                  tym_cost, tym_ta_times, -1, lowLevelNode1, -1, flush=True)
        #

