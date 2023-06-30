from matplotlib import pyplot as plt
from gym_duckiematrix import DuckietownEnv
import numpy as np

# vehicle calibration
# - camera
camera_info = {
    "width": 640,
    "height": 480
}


# # create matplot window
# window = plt.imshow(np.zeros((camera_info["height"], camera_info["width"], 3)))
# plt.axis("off")
# fig = plt.figure(1)
# plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)

env = DuckietownEnv()

obs, info = env.reset()

print(f"Observation: {obs}"
      f"\nInfo: {info}")
# rgb_img = obs[0]
# print(f"Image_shape: {rgb_img.shape}")
# window.set_data(rgb_img)
# fig.canvas.draw_idle()
# fig.canvas.start_event_loop(0.00001)

# print("Environment reset successfully!")
# pause()
obs, reward, truncated, terminated, info = env.step(env.action_space.sample())