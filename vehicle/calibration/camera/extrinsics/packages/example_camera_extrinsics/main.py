#!/usr/bin/env python3

# line detector
from threading import Thread
from typing import Dict
from typing import Union

import cv2
import matplotlib.pyplot as plt
import numpy as np
from turbojpeg import TurboJPEG

# duckiematrix
from dt_computer_vision.camera import CameraModel, BGRImage
from dt_computer_vision.camera.calibration.extrinsics.boards import CalibrationBoard8by6
from dt_computer_vision.camera.calibration.extrinsics.chessboard import find_corners
from dt_computer_vision.camera.calibration.extrinsics.exceptions import NoCornersFoundException
from dt_computer_vision.camera.calibration.extrinsics.ransac import estimate_homography
from dt_computer_vision.camera.calibration.extrinsics.rendering import draw_corners
from dt_duckiematrix_protocols import Matrix

# intrinsics camera calibration
camera_info: Dict[str, Union[np.ndarray, int]] = \
    {
        "width": 640,
        "height": 480,
        "K": np.reshape(
            [
                295.79606866959824,
                0.0,
                321.2621599038631,
                0.0,
                299.5389048862878,
                241.73616515312332,
                0.0,
                0.0,
                1.0,
            ],
            (3, 3)
        ),
        "D": [
            -0.23543978771661125,
            0.03637781479419574,
            -0.0033069818601306755,
            -0.0012140708179525926,
            0.0,
        ],
        "P": np.reshape(
            [
                201.14027404785156,
                0.0,
                319.5586620845679,
                0.0,
                0.0,
                239.74398803710938,
                237.60151004037834,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
            (3, 4)
        )
    }

# JPEG decoder
jpeg = TurboJPEG()
calibration_board = CalibrationBoard8by6

# create matplot window
window = plt.imshow(np.zeros((camera_info["height"], camera_info["width"], 3)))
plt.axis("off")
fig = plt.figure(1)
plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
plt.pause(0.01)


class CameraExtrinsicsCalibration:

    def __init__(self):
        # create connection to the matrix engine
        matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self.robot = matrix.robots.DB21M("map_0/vehicle_0")
        # create camera model
        self.camera = CameraModel(
            width=camera_info["width"],
            height=camera_info["height"],
            K=camera_info["K"],
            D=camera_info["D"],
            P=camera_info["P"]
        )
        self._is_shutdown: bool = False

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    def shutdown(self):
        self._is_shutdown = True

    @staticmethod
    def _show_frame(bgr: BGRImage):
        # bgr -> rgb
        rgb = bgr[:, :, [2, 1, 0]]
        # show frame
        window.set_data(rgb)
        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.001)

    def run(self):
        while not self.is_shutdown:
            cframe = self.robot.camera.capture(block=True, timeout=10)

            # get frame as uint8 array
            jpg = cframe.as_uint8()
            bgr = jpeg.decode(jpg)

            # rectify image
            bgr = self.camera.rectifier.rectify(bgr, interpolation=cv2.INTER_LINEAR)

            # find corners
            try:
                corners = find_corners(bgr, calibration_board)
                if len(corners) != (calibration_board.columns - 1) * (calibration_board.rows - 1):
                    raise RuntimeError("Not enough corners")
            except NoCornersFoundException:
                # show frame without corners
                self._show_frame(bgr)
                continue

            # estimate homography
            self.camera.H = estimate_homography(corners, calibration_board, self.camera)

            # render points
            corners_bgr = draw_corners(bgr, calibration_board, corners)

            # show frame with corners
            self._show_frame(corners_bgr)


if __name__ == "__main__":
    node = CameraExtrinsicsCalibration()

    def interaction():
        print("Press [ENTER] when you see a good board detection...")
        input()
        print(f"Estimated homography is:\n{node.camera.H}")
        node.shutdown()

    Thread(target=interaction).start()

    node.run()
