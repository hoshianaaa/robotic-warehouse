import gym
import rware

env = gym.make("rware-tiny-1ag-v1", sensor_range=3, request_queue_size=6)

#print(env.n_agents)
#print(env.action_space[0])
#print(env.observation_space)

obs = env.reset()  # a tuple of observations
print(obs)

actions = env.action_space.sample()  # the action space can be sampled
n_obs, reward, done, info = env.step(actions)
#print(actions)
#print(done)    # [False, False]
#print(reward)  # [0.0, 0.0]
#print(obs[0]["location"])

action_list = []
action_list.append((1))
#action_list.append((1,1))

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
  print(env.agents[0].dir.value)

  # shelf
  print()
  print("*** shelf ***")
  for i in range(len(env.shelfs)):
    print(i)
    print(env.shelfs[i].x)
    print(env.shelfs[i].y)
    if env.shelfs[i] in env.request_queue:
      print("req")
    else:
      print("not req")
  
  time.sleep(1)
  env.render()

env.close()
