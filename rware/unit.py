############## python utils ############3

import os
home_dir = os.environ['HOME']
python_util_dir = home_dir + "/python_utils"

import sys
sys.path.append(python_util_dir)

append_dir = python_util_dir + "/ros"
sys.path.append(append_dir)
from calculation import *

################ A start ###########

import os

from astar import *

###################################

import gym
import rware

env = gym.make("rware-tiny-1ag-v1", sensor_range=3, request_queue_size=1)

#print(env.n_agents)
#print(env.action_space[0])
#print(env.observation_space)
obs = env.reset()  # a tuple of observations
#env.agents[0].x = 1
#env.agents[0].y = 1

print(obs)

#print(actions)
#print(done)    # [False, False]
#print(reward)  # [0.0, 0.0]
#print(obs[0]["location"])

action_list = []
action_list.append((1))
#action_list.append((1,1))

# エージェントの位置を初期化

################## A Start #######################

sx = env.agents[0].x  # [m]
sy = env.agents[0].y  # [m]
gx = 10.0  # [m]
gy = 10.0  # [m]
grid_size = 1.0  # [m]
robot_radius = 0.1  # [m]

ox, oy = [], []
for i in range(-1, 11):
    ox.append(i)
    oy.append(-1.0)
for i in range(-1, 11):
    ox.append(11.0)
    oy.append(i)
for i in range(-1, 11):
    ox.append(i)
    oy.append(11.0)
for i in range(-1, 11):
    ox.append(-1.0)
    oy.append(i)

ox.append(11)
oy.append(11)

for i in range(len(env.shelfs)):
  print(i)
  print(env.shelfs[i].x)
  print(env.shelfs[i].y)
  if env.shelfs[i] in env.request_queue:
    print("req")
    gx = env.shelfs[i].x
    gy = env.shelfs[i].y
  else:
    ox.append(env.shelfs[i].x)
    oy.append(env.shelfs[i].y)

    print("not req")
 
if show_animation:  # pragma: no cover
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
rx, ry = a_star.planning(sx, sy, gx, gy)

rx.reverse()
ry.reverse()

print("**** result ****")
print(rx)
print(ry)

drx = []
dry = []
for i in range(len(rx)-1):
  drx.append(rx[i+1] - rx[i])
  dry.append(ry[i+1] - ry[i])

print("**** d result ****")
print(drx)
print(dry)


if show_animation:  # pragma: no cover
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()

#################################################
# **** convert a start path to action list **** #
#################################################

# dir 0: DOWN(0,-1), 1: UP(0,1), 2: LEFT(-1, 0), 3: RIGHT(1,0)

# *** action ***
#  NOOP = 0
#  FORWARD = 1
#  LEFT = 2
#  RIGHT = 3
#  TOGGLE_LOAD = 4

sx = env.agents[0].x  # [m]
sy = env.agents[0].y  # [m]
r_dir = env.agents[0].dir.value
print("****start****")
print("sx:", sx)
print("sy:", sy)
print("r_dir:", r_dir)

r_dir_vec = [0,0]

if r_dir == 0:
  r_dir_vec = [0,-1]
if r_dir == 1:
  r_dir_vec = [0,1]
if r_dir == 2:
  r_dir_vec = [-1,0]
if r_dir == 3:
  r_dir_vec = [1,0]


action_list = []
action_list_debug = []

print("rotate")
for i in range(len(rx)-1):

  target_vec = [drx[i], dry[i]]

  rotate = angle_between_vectors(r_dir_vec, target_vec)
  print(rotate)
  rotate = rotate / 90
  print(r_dir_vec, target_vec, rotate)
  r_dir_vec = target_vec
  
  if rotate > 0:
    for i in range(int(rotate)):
      # 左回転
      action_list.append((3,))
      action_list_debug.append("rotate-left")

  if rotate < 0:
    for i in range(int(-rotate)):
      # 左回転
      action_list.append((2,))
      action_list_debug.append("rotate-right")

  action_list.append((1,))
  action_list_debug.append("straight")

print(action_list_debug)
#################################################

import time
for i in range(len(action_list)):
#while True:

  actions = env.action_space.sample()  # the action space can be sampled

  # *** action ***
  #  NOOP = 0
  #  FORWARD = 1
  #  LEFT = 2
  #  RIGHT = 3
  #  TOGGLE_LOAD = 4
#  actions = (2,)
#  print(actions)
  n_obs, reward, done, info = env.step(action_list[i])
#  n_obs, reward, done, info = env.step(actions)
#  n_obs, reward, done, info = env.step((0))

#  print(reward)
#  print(n_obs)
  #print(info)
#  print(env.observation_space)
 # print(env.grid)
#  print(env.normalised_coordinates)
  #print(env.shelfs)
  #print(env.highways)

  # goal
  print(env.goals)

  # agent
  print(env.agents[0].x)
  print(env.agents[0].y)
  ## dir 0: 右(1,0), 1: 左(-1,0), 2: 上(0,1), 3:下(0,-1)

  # shelf
#  print()
#  print("*** shelf ***")
#  for i in range(len(env.shelfs)):
#    print(i)
#    print(env.shelfs[i].x)
#    print(env.shelfs[i].y)
#    if env.shelfs[i] in env.request_queue:
#      print("req")
#    else:
#      print("not req")
  
#  print(env.request_queue)
  
  env.render()
  print(env.agents[0].dir.value)
  time.sleep(1)

env.close()
