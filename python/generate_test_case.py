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
    # parser.add_argument("--load_size", type=int, help="load_size")
    parser.add_argument("--common_size", type=int, help="common_size")
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)

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
    test_yaml_map_data = {'dimensions': [height, width], 'obstacles': []}
    for i in range(height):
        for j in range(width):
            if map_data[i][j] == 0:
                good_points.append([i, j])
            else:
                test_yaml_map_data['obstacles'].append([i, j])

    good_points = np.array(good_points, dtype=int)
    # max_agent_num = 20
    for agent_num in range(21, 52, 5):
        for test_idx in range(20):
            args.common_size = agent_num//4
            args.other_selected_goal_num = agent_num-args.common_size
            random_idx = np.random.choice(good_points.shape[0], agent_num * 2 + args.common_size, replace=False)
            init_startP_goalP = good_points[random_idx]
            agent_startP = init_startP_goalP[:agent_num]
            agents_goalP = []
            for i in range(agent_num):
                x = {}
                p = HashPoint(init_startP_goalP[i+agent_num][0], init_startP_goalP[i+agent_num][1], width)
                x[p] = 1
                agents_goalP.append(x)

            agent_commonP = init_startP_goalP[agent_num*2:]
            for i in range(agent_num):
                for j in range(args.common_size):
                    p = HashPoint(agent_commonP[j][0], agent_commonP[j][1], width)
                    agents_goalP[i][p] = 1

            other_points = good_points[~np.isin(np.arange(good_points.shape[0]), random_idx)]
            other_selected_goal_num = args.other_selected_goal_num
            random_idx = np.random.choice(other_points.shape[0], other_selected_goal_num)
            goal_points = other_points[random_idx]
            for i in range(other_selected_goal_num):
                agent_idx = np.random.randint(0, agent_num)
                p = HashPoint(goal_points[i][0], goal_points[i][1], width)
                agents_goalP[agent_idx][p] = 1

            for i in range(agent_num):
                agents_goalP[i] = agents_goalP[i].keys()
                agents_goalP[i] = [point.tolist() for point in agents_goalP[i]]


            test_file = {"map": dp(test_yaml_map_data)}
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