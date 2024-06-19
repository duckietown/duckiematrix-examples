#!/usr/bin/env python3

from typing import Dict

import matplotlib.pyplot as plt
import numpy as np
from turbojpeg import TurboJPEG

from duckietown.sdk.robots.duckiebot import DB21J

import logging
logging.getLogger("matplotlib.pyplot").setLevel(logging.WARNING)
logging.getLogger("matplotlib.font_manager").setLevel(logging.WARNING)
logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)

# vehicle calibration
# - camera
camera_info: Dict[str, int] = {
    "width": 640,
    "height": 480
}

# JPEG decoder
jpeg = TurboJPEG()

# create matplot window
window = plt.imshow(np.zeros((camera_info["height"], camera_info["width"], 3)))
plt.axis("off")
fig = plt.figure(1)
plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
plt.pause(0.01)


class CameraIO:

    def __init__(self):
        # create connection to the matrix engine
        self.robot: DB21J = DB21J("map_0/vehicle_0", simulated=True)

    def run(self):
        self.robot.camera.start()
        while not self.is_shutdown:
            bgr: np.ndarray = self.robot.camera.capture(block=True)
            # bgr -> rgb
            rgb = bgr[:, :, [2, 1, 0]]
            # show frame
            window.set_data(rgb)
            fig.canvas.draw_idle()
            fig.canvas.start_event_loop(0.00001)
        self.robot.camera.stop()

    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False


if __name__ == "__main__":
    node = CameraIO()
    node.run()
