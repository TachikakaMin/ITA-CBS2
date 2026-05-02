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
    parser.add_argument("--common_ratio", help="map output")
    parser.add_argument("--agent_start", type=int, default=15)
    parser.add_argument("--agent_stop", type=int, default=60)
    parser.add_argument("--agent_step", type=int, default=5)
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
    good_points, component_id, components = connected_component_points(map_data)
    common_goals = int(total_goals * common_ratio)
    unique_goals = total_goals - common_goals
    min_component_size = total_goals + 1
    eligible_points = []
    for point in good_points:
        cid = component_id[point[0]][point[1]]
        if components[cid].shape[0] >= min_component_size:
            eligible_points.append(point)
    good_points = np.array(eligible_points, dtype=int)

    for agent_num in tqdm(range(args.agent_start, args.agent_stop + 1, args.agent_step)):
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
                common_idx = np.random.choice(
                    comp_points.shape[0], common_goals, replace=False)
                agent_commonP = comp_points[common_idx]
                local_pool = np.delete(comp_points, common_idx, axis=0)

                for agent_id in agent_ids:
                    random_idx = np.random.choice(
                        local_pool.shape[0], 1 + unique_goals, replace=False)
                    agent_localP = local_pool[random_idx]
                    init_goal = agent_localP[0]
                    p = HashPoint(init_goal[0], init_goal[1], width)
                    agents_goalP[agent_id][p] = 1
                    for goal in agent_commonP:
                        p = HashPoint(goal[0], goal[1], width)
                        agents_goalP[agent_id][p] = 1
                    agent_uniqueP = agent_localP[1:1 + unique_goals]
                    for goal in agent_uniqueP:
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
