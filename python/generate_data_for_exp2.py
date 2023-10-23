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
from tqdm import tqdm
import shutil

class HashPoint():
    def __init__(self, x, y, width):
        self.x = x
        self.y = y
        self.width = width

    def __hash__(self):
        return int(self.x * self.width + self.y)

    def tolist(self):
        return [int(self.x), int(self.y)]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_path", help="input file containing map")
    parser.add_argument("--output_dir", help="map output")
    parser.add_argument("--common_ratio", help="map output")
    # parser.add_argument("--load_size", type=int, help="load_size")
    args = parser.parse_args()
    args.common_ratio = float(args.common_ratio)
    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    shutil.copy(args.map_path, args.output_dir)
    test_file = {"map" : os.path.basename(args.map_path)}
    np.random.seed(1)
    map_file_path = args.map_path
    map_data = []
    with open(map_file_path) as map_file:
        now_map = map_file.readlines()
        height = now_map[1].replace('\n', "").split(" ")[1]
        height = int(height)
        width = now_map[2].replace('\n', "").split(" ")[1]
        width = int(width)
        for i in range(height):
            x = now_map[i + 4].replace("\n", "").replace(".", "0").replace("@", "1").replace("T", "1")
            x = list(map(int, x))
            map_data.append(x)
    map_data = np.array(map_data)

    good_points = []
    # test_yaml_map_data = {'dimensions': [height, width], 'obstacles': obstacles}
    for i in range(height):
        for j in range(width):
            if map_data[i][j] == 0:
                good_points.append([i, j])

    good_points = np.array(good_points, dtype=int)

    common_ratio = args.common_ratio
    # 15 for empty 32*32 15-60
    # 15 for random 32*32 15-60
    # 80 for warehouse 161*63 15-60
    # 40 for den312d 65*81 15-60
    # 15 for maze 32*32_2 5-35
    # 50 for room 64*64_8, 15-60
    # 20 for orz900d 1491*656, 15-60
    # 20 for Boston 256*256 15-60
    total_goals = 15
    for agent_num in tqdm(range(15, 65, 5)):
        for test_idx in range(20):
            random_idx = np.random.choice(good_points.shape[0], agent_num*2, replace=False)
            agent_startP = good_points[random_idx[:agent_num]]
            agent_initGoalP = good_points[random_idx[agent_num:]]

            agents_goalP = []
            for i in range(agent_num):
                x = {}
                p = HashPoint(agent_initGoalP[i][0], agent_initGoalP[i][1], width)
                x[p] = 1
                agents_goalP.append(x)

            common_goals = int(total_goals * common_ratio)
            random_idx = np.random.choice(good_points.shape[0], common_goals, replace=False)
            agent_commonP = good_points[random_idx]

            for i in range(agent_num):
                for j in range(common_goals):
                    p = HashPoint(agent_commonP[j][0], agent_commonP[j][1], width)
                    agents_goalP[i][p] = 1

            other_points = good_points[~np.isin(np.arange(good_points.shape[0]), random_idx)]
            unique_goals = total_goals - common_goals
            for i in range(agent_num):
                random_idx = np.random.choice(other_points.shape[0], unique_goals, replace=False)
                agent_uniqueP = other_points[random_idx]
                other_points = other_points[~np.isin(np.arange(other_points.shape[0]), random_idx)]
                for j in range(unique_goals):
                    p = HashPoint(agent_uniqueP[j][0], agent_uniqueP[j][1], width)
                    agents_goalP[i][p] = 1

            for i in range(agent_num):
                agents_goalP[i] = agents_goalP[i].keys()
                agents_goalP[i] = [point.tolist() for point in agents_goalP[i]]


            # test_file = {"map": dp(test_yaml_map_data)}

            test_file["agents"] = []
            for idx, agent_goalPs in enumerate(agents_goalP):
                agent = {}
                agent["name"] = "agent" + str(idx)
                agent["start"] = agent_startP[idx].tolist()
                agent["potentialGoals"] = agents_goalP[idx]
                test_file["agents"].append(agent)
            file_name = os.path.basename(map_file_path)
            file_name = os.path.splitext(file_name)[0]+ "_agents_" + str(agent_num) + "_test_" + str(test_idx) + ".yaml"
            output_file_path = os.path.join(args.output_dir, file_name)
            with open(output_file_path, 'w') as outfile:
                yaml.safe_dump(
                    json.loads(
                        json.dumps(test_file)
                    ), outfile, default_flow_style=None
                )