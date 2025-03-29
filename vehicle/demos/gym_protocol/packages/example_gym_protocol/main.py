from gym_duckiematrix.DB21J import DuckiematrixDB21JEnv
from time import sleep

env = DuckiematrixDB21JEnv()

obs, info = env.reset()

for i in range(100):
    action = env.action_space.sample()
    print(f"sending action {action}")
    obs, reward, truncated, terminated, info = env.step(action)
    print("received observation")
    sleep(0.1)
    #env.render()