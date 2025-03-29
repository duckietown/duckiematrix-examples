from gym_duckiematrix.DB21J import DuckiematrixDB21JEnv
from time import sleep

env = DuckiematrixDB21JEnv()

obs, info = env.reset()

for i in range(100):
    action = env.action_space.sample()
    obs, reward, truncated, terminated, info = env.step(action)
    sleep(0.1)
