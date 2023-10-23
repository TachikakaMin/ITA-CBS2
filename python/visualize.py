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
import tqdm
Colors = ['orange', 'blue', 'white', 'purple']

def load_map(file_path):
  with open(file_path, 'r') as f:
    lines = f.readlines()

  # Skip the metadata lines
  map_lines = lines[4:]
  # Convert map characters to numbers
  map_data = [[0 if cell == '.' else 1 for cell in line.strip()] for line in map_lines]
  # Convert to numpy array
  map_array = np.array(map_data)

  return map_array

class Animation:
  def __init__(self, map, schedule, args):
    self.map = map
    self.schedule = schedule
    aspect = 1
    if isinstance(map["map"], dict):
      aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
    else:
      mp_path = map["map"]
      dir_path = os.path.dirname(args.map)
      mp_path = os.path.join(dir_path, mp_path)
      self.map_data = load_map(mp_path)
      aspect = self.map_data.shape[0] / self.map_data.shape[1]
    self.fig = plt.figure(frameon=False, figsize=(10 * aspect, 10))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    if isinstance(map["map"], dict):
      xmax = map["map"]["dimensions"][0] - 0.5
      ymax = map["map"]["dimensions"][1] - 0.5
    else:
      xmax = self.map_data.shape[0] - 0.5
      ymax = self.map_data.shape[1] - 0.5

    # self.ax.relim()
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # self.ax.set_xticks([])
    # self.ax.set_yticks([])
    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('off')

    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    if isinstance(map["map"], dict):
      for o in map["map"]["obstacles"]:
        x, y = o[0], o[1]
        self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))
    else:
      indices = np.where(self.map_data != 0)
      for i in tqdm.tqdm(range(len(indices[0]))):
        x = indices[0][i]
        y = indices[1][i]
        self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))

    # create agents:
    self.T = 0
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      if "goal" in d:
        goals = [d["goal"]]
      if "potentialGoals" in d:
        goals = [goal for goal in d["potentialGoals"]]
      for goal in goals:
        self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[i%len(Colors)], edgecolor='black', alpha=0.5))

    total_cost = 0
    total_cost2 = 0
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      name = d["name"]
      self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.3, facecolor=Colors[i%len(Colors)], edgecolor='black')
      self.agents[name].original_face_color = Colors[i%len(Colors)]
      self.patches.append(self.agents[name])
      total_cost += len(schedule["schedule"][name]) - 1
      total_cost2 += schedule["schedule"][name][-1]["t"]
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])
    print(total_cost, total_cost2)


    # self.ax.set_axis_off()
    # self.fig.axes[0].set_visible(False)
    # self.fig.axes.get_yaxis().set_visible(False)

    # self.fig.tight_layout()
    print("frames: ", int(self.T+1) * 10)
    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * 10,
                               interval=100,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    for agent_name in self.schedule["schedule"]:
      agent = schedule["schedule"][agent_name]
      pos = self.getState(i / 10, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)

    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 0.7:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=2, help="speedup-factor")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.safe_load(map_file)

  with open(args.schedule) as states_file:
    schedule = yaml.safe_load(states_file)

  animation = Animation(map, schedule, args)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()
