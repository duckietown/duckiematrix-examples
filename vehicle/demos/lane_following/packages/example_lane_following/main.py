#!/usr/bin/env python3

from collections import defaultdict
from queue import Queue, Empty
from threading import Thread
from typing import Any, Union, Tuple
# line detector
from typing import Dict
# ground projector
from typing import List

import numpy as np
import time
from turbojpeg import TurboJPEG

from dt_computer_vision.camera import CameraModel, NormalizedImagePoint, Pixel
from dt_computer_vision.ground_projection import GroundProjector
from dt_computer_vision.ground_projection.rendering import draw_grid_image, debug_image
from dt_computer_vision.ground_projection.types import GroundPoint
from dt_computer_vision.line_detection import LineDetector, ColorRange, Detections
# lane filter
from dt_computer_vision.line_detection.rendering import draw_segments
from dt_duckiematrix_messages.CameraFrame import CameraFrame
# duckiematrix
from dt_duckiematrix_protocols import Matrix
# inverse kinematics
from dt_modeling.kinematics.inverse import InverseKinematics
# lane controller
from dt_motion_planning.lane_controller import PIDLaneController
from dt_state_estimation.lane_filter import LaneFilterHistogram
from dt_state_estimation.lane_filter.types import Segment, SegmentColor, SegmentPoint

# types
Color = Tuple[int, int, int]

# vehicle calibration
# - camera
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
        ),
        "H": np.reshape(
            [
                8.56148231e-03,
                2.22480148e-01,
                4.24318934e-01,
                -5.67022044e-01,
                -1.13258040e-03,
                6.81113839e-04,
                5.80917161e-02,
                4.35079347e+00,
                1.0],
            (3, 3)
        ),
    }
crop_top = 240
image_crop = [0, crop_top, 640, camera_info["height"] - crop_top]

# - distance between wheels
wheel_baseline: float = 0.1
# - wheel radius
wheel_radius = 0.0318
# - IK parameters
v_max: float = 1.0
omega_max: float = 0.8
gain: float = 1.0

# last command
last_command = (0.0, 0.0)
last_command_time = 0.0

# colors
color_ranges: Dict[str, ColorRange] = {
    "white": ColorRange.fromDict({
        "low": [0, 0, 150],
        "high": [180, 100, 255]
    }),
    "yellow": ColorRange.fromDict({
        "low": [0, 100, 100],
        "high": [45, 255, 255]
    })
}
colors: Dict[str, Color] = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "white": (255, 255, 255),
}

# JPEG decoder
jpeg = TurboJPEG()

import matplotlib.pyplot as plt

# create matplot window
# window = plt.imshow(np.zeros((camera_info["width"] - crop_top, 640, 3)))
window = plt.imshow(np.zeros((400, 400, 3)))
plt.axis("off")
fig = plt.figure(1)
plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
plt.pause(0.01)


# # new frame callback
# def on_new_frame(cframe: CameraFrame):
#     # get frame as uint8 array
#     jpeg = cframe.as_uint8()
#     # uint8 array to bgr8
#     rgb = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
#     bgr = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
#


class LaneFollowing:

    def __init__(self):
        # create connection to the matrix engine
        matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self.robot = matrix.robots.DB21M("map_0/vehicle_0")
        self.robot.camera.attach(self._img_cb)
        # apply crop to camera calibration
        self._camera_info = camera_info
        x, y, w, h = image_crop
        # - update K
        _K = self._camera_info["K"]
        _K[0][2] = _K[0][2] - x
        _K[1][2] = _K[1][2] - y
        # - update P
        _P = self._camera_info["P"]
        _P[0][2] = _P[0][2] - x
        _P[1][2] = _P[1][2] - y
        # thread pool
        self._queues = defaultdict(lambda: Queue(1))
        # make worker threads
        self._workers = [
            Thread(target=self._line_detector, daemon=True),
            Thread(target=self._lane_filter, daemon=True),
            Thread(target=self._lane_controller, daemon=True),
            Thread(target=self._inverse_kinematics, daemon=True),
            Thread(target=self._wheels, daemon=True),
            # Thread(target=self._command_loop, daemon=True),
        ]
        # start worker threads
        for w in self._workers:
            w.start()

    def join(self):
        while not self.is_shutdown:
            # bgr = self._queue_pop("lines_image")
            bgr = self._queue_pop("segments_image")
            rgb = bgr[:, :, [2, 1, 0]]
            # show frame
            window.set_data(rgb)
            fig.canvas.draw_idle()
            fig.canvas.start_event_loop(0.001)

    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False

    def _queue_pop(self, queue: str) -> Any:
        value = self._queues[queue].get()
        # self.loginfo(f"POP: {queue}")
        return value

    def _queue_full(self, queue: str):
        return self._queues[queue].full()

    def _queue_put(self, queue: str, value: Any):
        try:
            self._queues[queue].get(block=False)
        except Empty:
            pass
        self._queues[queue].put(value)
        # self.loginfo(f"PUT: {queue}")

    def _img_cb(self, cframe: CameraFrame):
        if self._queue_full("images"):
            return
        # get frame as uint8 array
        jpg = cframe.as_uint8()
        bgr = jpeg.decode(jpg)
        # crop frame
        x, y, w, h = image_crop
        bgr = bgr[y:y + h, x:x + w, :]
        # senf frame to line detector
        self._queue_put("image_plot", bgr)
        self._queue_put("image", bgr)

    def _line_detector(self):
        color_order = ["yellow", "white"]
        colors_to_detect = [color_ranges[c] for c in color_order]
        detector = LineDetector()
        while not self.is_shutdown:
            bgr = self._queue_pop("image")
            color_detections: List[Detections] = detector.detect(bgr, colors_to_detect)
            lines: Dict[str, dict] = {}
            for i, detections in enumerate(color_detections):
                color = color_order[i]
                # pack detections in a dictionary
                lines[color] = {
                    "lines": detections.lines.tolist(),
                    "centers": detections.centers.tolist(),
                    "normals": detections.normals.tolist(),
                    "color": color_ranges[color].representative
                }
            self._queue_put("lines", lines)

            # draw detections on top of the image
            image_w_dets = draw_segments(bgr, {color_ranges["yellow"]: color_detections[0]})
            self._queue_put("lines_image", image_w_dets)

    def _lane_filter(self):
        global last_command_time
        # ---
        camera = CameraModel(
            width=self._camera_info["width"],
            height=self._camera_info["height"],
            K=self._camera_info["K"],
            D=self._camera_info["D"],
            P=self._camera_info["P"],
            H=self._camera_info["H"],
        )
        projector = GroundProjector(camera)
        # create filter
        filter = LaneFilterHistogram()

        grid = draw_grid_image((400, 400))

        while not self.is_shutdown:
            segments: List[Segment] = []
            colored_segments: Dict[Color, List[Tuple[GroundPoint, GroundPoint]]] = {}

            lines = self._queue_pop("lines")
            if self._queue_full("segments"):
                continue
            for color, colored_lines in lines.items():
                grounded_segments: List[Tuple[GroundPoint, GroundPoint]] = []
                for line in colored_lines["lines"]:
                    # distorted pixels
                    p0: Pixel = Pixel(line[0], line[1])
                    p1: Pixel = Pixel(line[2], line[3])
                    # distorted pixels to rectified pixels
                    p0_rect: Pixel = camera.rectifier.rectify_pixel(p0)
                    p1_rect: Pixel = camera.rectifier.rectify_pixel(p1)
                    # rectified pixel to normalized coordinates
                    p0_norm: NormalizedImagePoint = camera.pixel2vector(p0_rect)
                    p1_norm: NormalizedImagePoint = camera.pixel2vector(p1_rect)
                    # project image point onto the ground plane
                    grounded_p0: SegmentPoint = projector.vector2ground(p0_norm)
                    grounded_p1: SegmentPoint = projector.vector2ground(p1_norm)
                    # add grounded segment to output
                    segments.append(Segment(
                        points=[grounded_p0, grounded_p1],
                        color=SegmentColor(color)
                    ))
                    grounded_segments.append((grounded_p0, grounded_p1))

                colored_segments[colors[color]] = grounded_segments

            # draw segments
            image_w_segs = debug_image(colored_segments, (400, 400), background_image=grid)
            self._queue_put("segments_image", image_w_segs)

            # apply update
            filter.update(segments)
            # predict
            if last_command_time:
                delta_t: float = time.time() - last_command_time
                filter.predict(delta_t, *last_command)
                last_command_time = None
            # get new estimate
            d_hat, phi_hat = filter.get_estimate()
            self._queue_put("d_phi", (d_hat, phi_hat))

    def _lane_controller(self):
        global last_command
        # ---
        controller = PIDLaneController(k_theta=-2.5)
        while not self.is_shutdown:
            d_hat, phi_hat = self._queue_pop("d_phi")
            controller.update(d_hat, phi_hat, time.time())
            v, w = controller.compute_commands()
            self._queue_put("v_w", (v, w))
            last_command = (v, w)

    def _inverse_kinematics(self):
        ik = InverseKinematics(
            wheel_baseline=wheel_baseline,
            wheel_radius=wheel_radius,
            v_max=v_max,
            omega_max=omega_max
        )
        while not self.is_shutdown:
            v, w = self._queue_pop("v_w")
            wl, wr = ik.get_wheels_speed(v, w)
            self._queue_put("wl_wr", (wl, wr))

    def _wheels(self):
        global last_command_time
        # ---
        while not self.is_shutdown:
            wl, wr = self._queue_pop("wl_wr")
            with self.robot.session():
                self.robot.drive(wl, wr)
                last_command_time = time.time()


if __name__ == "__main__":
    node = LaneFollowing()
    node.join()
