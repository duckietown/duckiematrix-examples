from gym_duckiematrix import DuckietownEnv

env = DuckietownEnv()

obs, info = env.reset()

for i in range(1000):
    obs, reward, truncated, terminated, info = env.step(env.action_space.sample())
    env.render()