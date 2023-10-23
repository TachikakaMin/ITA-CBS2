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
    parser.add_argument("--file_path", help="input file")
    args = parser.parse_args()
    file_path = args.file_path
    with open(file_path) as data_file:
        data = data_file.readlines()
        data = [line.replace("\n","").split(" ") for line in data]

    tym_ta_time = []
    tym_time = []
    cbsta_time = []
    test_name = []
    agent_num = []
    for line in data:
        x, y, z, a, _ = line
        test_name.append(x)
        tmp = x.split("_")[2]
        agent_num.append(int(tmp))
        if not ("out" in y):
            tym_ta_time.append(eval(y))
        else :
            tym_ta_time.append(40)

        if not ("out" in z):
            tym_time.append(eval(z))
        else :
            tym_time.append(40)

        if not ("out" in a):
            cbsta_time.append(eval(a))
        else :
            cbsta_time.append(40)

    tym_ta_time = np.array(tym_ta_time)
    tym_time = np.array(tym_time)
    cbsta_time = np.array(cbsta_time)
    agent_num = np.array(agent_num)

    result = {}
    for i in range(len(tym_ta_time)):
        idx = agent_num[i]

        if not (idx in result.keys()):
            result[idx] = {}
        tmp_keys = ['cnt', 'total_time', '']
        if not ("cnt" in result.keys()):
            result[idx]["cnt"] = 0

        if not ("total" in result.keys()):
            result[idx]["total"] = 0

