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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", help="input file containing map")
    parser.add_argument("output_dir", help="map output")
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)

    map_file_paths = []
    if os.path.isdir(args.map_dir):
        map_file_paths = [f for f in glob.glob(args.map_dir + "/*.yaml")]
    else:
        map_file_paths.append(args.map_dir)

    for map_file_path in map_file_paths:
        with open(map_file_path) as map_file:
            now_map = yaml.safe_load(map_file)
        all_goals = []
        # print(now_map)
        for agent in now_map["agents"]:
            all_goals.append(dp(agent['goal']))
            del agent['goal']

        for i,agent in enumerate(now_map["agents"]):
            now_map["agents"][i]["potentialGoals"] = all_goals

        file_name = os.path.basename(map_file_path)
        output_file_path = os.path.join(args.output_dir, file_name)
        with open(output_file_path, 'w') as outfile:
            yaml.safe_dump(
                json.loads(
                    json.dumps(now_map)
                ), outfile, default_flow_style=None
            )
            # yaml.dump(now_map, outfile, default_flow_style=None)
        # aaaa