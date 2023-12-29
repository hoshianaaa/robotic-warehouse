
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

if show_animation:  # pragma: no cover
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()

#################################################

import time
#for i in range(len(action_list)):
while True:

  actions = env.action_space.sample()  # the action space can be sampled
  actions = (2,)
#  print(actions)
  #n_obs, reward, done, info = env.step(action_list[i])
  n_obs, reward, done, info = env.step(actions)
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
  ## dir 0: 右, 1: 左, 2: 上, 3:下
  print(env.agents[0].dir.value)

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
  
  time.sleep(1)
  env.render()

env.close()
