#!/usr/bin/env python3

import time
from collections import defaultdict
from queue import Queue, Empty
from threading import Thread
from typing import Any, Tuple, Dict, List

import numpy as np

import matplotlib.pyplot as plt
from turbojpeg import TurboJPEG

from dt_duckiematrix_protocols import Matrix


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
        matrix = Matrix("localhost")
        # create connection to the vehicle
        self.robot = matrix.robots.DB21M("map_0/vehicle_0")

    def run(self):
        while not self.is_shutdown:
            cframe = self.robot.camera.capture(block=True)
            # get frame as uint8 array
            jpg = cframe.as_uint8()
            bgr = jpeg.decode(jpg)
            # bgr -> rgb
            rgb = bgr[:, :, [2, 1, 0]]
            # show frame
            window.set_data(rgb)
            fig.canvas.draw_idle()
            fig.canvas.start_event_loop(0.00001)

    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False


if __name__ == "__main__":
    node = CameraIO()
    node.run()
