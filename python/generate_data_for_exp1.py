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
import shutil
from collections import deque

class HashPoint():
    def __init__(self, x, y, width):
        self.x = x
        self.y = y
        self.width = width

    def __hash__(self):
        return int(self.x * self.width + self.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.width == other.width

    def tolist(self):
        return [int(self.x), int(self.y)]


def connected_component_points(map_data):
    height, width = map_data.shape
    seen = np.zeros((height, width), dtype=bool)
    component_id = -np.ones((height, width), dtype=int)
    components = []
    good_points = []
    for i in range(height):
        for j in range(width):
            if map_data[i][j] != 0 or seen[i][j]:
                continue
            component = []
            cid = len(components)
            q = deque([(i, j)])
            seen[i][j] = True
            while q:
                x, y = q.popleft()
                component.append([x, y])
                component_id[x][y] = cid
                good_points.append([x, y])
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if (
                        0 <= nx < height
                        and 0 <= ny < width
                        and map_data[nx][ny] == 0
                        and not seen[nx][ny]
                    ):
                        seen[nx][ny] = True
                        q.append((nx, ny))
            components.append(np.array(component, dtype=int))
    return np.array(good_points, dtype=int), component_id, components

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_path", help="input file containing map")
    parser.add_argument("--output_dir", help="map output")
    # parser.add_argument("--load_size", type=int, help="load_size")
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    shutil.copy(args.map_path, args.output_dir)
    test_file = {"map" : os.path.basename(args.map_path)}

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

    good_points, component_id, components = connected_component_points(map_data)

    group_size = 5
    for agent_num in range(10, 205, 10):
        for test_idx in range(20):
            random_idx = np.random.choice(good_points.shape[0], agent_num, replace=False)
            agent_startP = good_points[random_idx]
            agents_goalP = []
            for i in range(agent_num):
                x = {}
                agents_goalP.append(x)

            component_agents = {}
            for i in range(agent_num):
                cid = component_id[agent_startP[i][0]][agent_startP[i][1]]
                component_agents.setdefault(cid, []).append(i)

            for cid, agent_ids in component_agents.items():
                comp_points = components[cid]
                if comp_points.shape[0] < len(agent_ids):
                    raise RuntimeError("component has fewer points than sampled starts")
                random_idx = np.random.choice(comp_points.shape[0], len(agent_ids), replace=False)
                component_goalP = comp_points[random_idx]
                for i in range(0, len(agent_ids), group_size):
                    group_agent_ids = agent_ids[i:i + group_size]
                    group_goalP = component_goalP[i:i + len(group_agent_ids)]
                    for agent_id in group_agent_ids:
                        for goal in group_goalP:
                            p = HashPoint(goal[0], goal[1], width)
                            agents_goalP[agent_id][p] = 1
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
