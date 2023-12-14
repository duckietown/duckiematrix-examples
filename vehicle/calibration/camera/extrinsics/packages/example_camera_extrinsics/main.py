#!/usr/bin/env python3
from typing import Dict, Union
from typing import List
from typing import Optional

import cv2
import numpy as np
import yaml
from dt_duckiematrix_protocols import Matrix
from turbojpeg import TurboJPEG, TJPF_BGR

from dt_computer_vision.camera import CameraModel, Pixel, BGRImage
from dt_computer_vision.camera import NormalizedImagePoint
from dt_computer_vision.camera.calibration.extrinsics.boards import CalibrationBoard8by6, CalibrationBoard
from dt_computer_vision.camera.calibration.extrinsics.chessboard import compute_homography_maps
from dt_computer_vision.camera.calibration.extrinsics.chessboard import find_corners, \
    get_ground_corners_and_error
from dt_computer_vision.camera.calibration.extrinsics.exceptions import NoCornersFoundException
from dt_computer_vision.camera.calibration.extrinsics.ransac import estimate_homography
from dt_computer_vision.camera.calibration.extrinsics.rendering import draw_corners, \
    top_view_projected_corners, draw_gui, GUI_RIGHT_IMAGE_ROI, GUI_BTN1_ROI, GUI_BTN2_ROI, GUI_SIZE, \
    VALIDATION_GUI_BTN1_ROI, draw_validation_gui, VALIDATION_GUI_RIGHT_IMAGE_ROI
from dt_computer_vision.camera.homography import HomographyToolkit, ResolutionIndependentHomography, \
    ResolutionDependentHomography
from dt_computer_vision.camera.types import Point, RegionOfInterest
from dt_computer_vision.ground_projection.types import GroundPoint

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


class CameraExtrinsicsCalibration:

    def __init__(self, quiet: bool = True):
        # create connection to the matrix engine
        matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self._robot = matrix.robots.DB21M("map_0/vehicle_0")
        # create camera model
        self._camera = CameraModel(
            width=camera_info["width"],
            height=camera_info["height"],
            K=camera_info["K"],
            D=camera_info["D"],
            P=camera_info["P"]
        )
        self._is_shutdown: bool = False
        self._quiet: bool = quiet
        # board to use
        self._board: CalibrationBoard = CalibrationBoard8by6
        # jpeg decoder
        self._jpeg = TurboJPEG()
        # store homography
        self._H: Optional[np.ndarray] = None
        self._error: Optional[float] = None
        # create window
        self._window = "Camera Extrinsic Calibration"
        cv2.namedWindow(self._window, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(self._window, *GUI_SIZE)
        cv2.setMouseCallback(self._window, self._gui_mouse_click)

    @property
    def H(self) -> Optional[np.ndarray]:
        return self._H

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    def shutdown(self):
        self._is_shutdown = True

    def run(self):
        while not self.is_shutdown:
            cframe = self._robot.camera.capture(block=True, timeout=1)

            # no frame?
            if cframe is None:
                # display empty GUI
                gui = draw_gui(False, None, None, 0, self._board, None, self._error)
                cv2.imshow(self._window, gui)
                cv2.waitKey(1)
                continue

            # get frame as uint8 array
            jpg = cframe.as_uint8()

            # decode JPEG
            distorted: BGRImage = self._jpeg.decode(jpg, pixel_format=TJPF_BGR)

            # rectify image
            rectified: BGRImage = self._camera.rectifier.rectify(distorted, interpolation=cv2.INTER_CUBIC)

            # find corners
            corners: List[Pixel] = []
            try:
                corners = find_corners(rectified, self._board, win_size=3)
                board_found = True
            except NoCornersFoundException:
                board_found = False

            if board_found:
                # estimate homography
                H: np.ndarray = estimate_homography(corners, self._board, self._camera)
                self._H = H

                # draw detected corners on top of the image
                image_w_corners: BGRImage = draw_corners(rectified, self._board, corners)

                # reproject all corners found in the image onto the ground plane and compute the errors
                image_corners: List[NormalizedImagePoint]
                ground_corners: List[GroundPoint]
                ground_corners_projected: List[GroundPoint]
                errors: List[float]

                image_corners, ground_corners, _, errors = \
                    get_ground_corners_and_error(self._camera, corners, self._board, H)
                assert len(errors) == len(corners)

                # compute average error
                avg_error = np.average(errors)
                std_error = np.std(errors)
                if not self._quiet:
                    print(f"Board detected, overall error: {avg_error:.4f}m +/- {std_error:.4f}m")

                # compute best error
                best_error: float = min(avg_error, self._error or avg_error)
                self._error = best_error

                # create re-projection image
                _, _, rw, rh = GUI_RIGHT_IMAGE_ROI
                right = top_view_projected_corners(ground_corners, errors, (rw, rh), start_y=0.15)

                # create GUI image
                gui = draw_gui(True, image_w_corners, right, len(corners), self._board, avg_error, best_error)
            else:
                # create GUI image
                active: bool = self._H is not None
                gui = draw_gui(active, rectified, None, 0, self._board, None, self._error)

            # display GUI
            cv2.imshow(self._window, gui)
            cv2.waitKey(1)

        # destroy window and exit
        cv2.destroyAllWindows()

    def _save_calibration(self):
        if self._H is None:
            return
        # stop printing
        self._quiet = True
        # print calibration
        print(f"""
Extrinsics calibration:
-----------------------

{yaml.safe_dump({"homography": self._H.flatten().tolist()})}
        """)
        # convert to resolution independent
        H_rd: ResolutionDependentHomography = ResolutionDependentHomography.read(self._H)
        H_ri: ResolutionIndependentHomography = H_rd.camera_independent(self._camera)
        # save to disk
        HomographyToolkit.save_to_disk(H_ri, "./default.yaml")
        # exit
        self._quit()

    def _quit(self):
        self._is_shutdown = True

    def _gui_mouse_click(self, event, x, y, _, __):
        # check if left mouse click
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # check which button was pressed
        btn_to_fcn = {
            GUI_BTN2_ROI: self._save_calibration,
            GUI_BTN1_ROI: self._quit,
        }

        for btn, fcn in btn_to_fcn.items():
            bx, by, bw, bh = btn
            if bx < x < bx + bw and by < y < by + bh:
                fcn()
                return


class CameraExtrinsicsValidation:

    def __init__(self, H: np.ndarray, pixels_per_meter: int = 800):
        # create connection to the matrix engine
        matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self._robot = matrix.robots.DB21M("map_0/vehicle_0")
        # create camera model
        self._camera = CameraModel(
            width=camera_info["width"],
            height=camera_info["height"],
            K=camera_info["K"],
            D=camera_info["D"],
            P=camera_info["P"]
        )
        self._is_shutdown: bool = False
        self._ppm: int = pixels_per_meter
        # jpeg decoder
        self._jpeg = TurboJPEG()
        # store homography
        self._H: np.ndarray = H
        # create window
        self._window = "Camera Extrinsic Validation"
        cv2.namedWindow(self._window, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(self._window, *GUI_SIZE)
        cv2.setMouseCallback(self._window, self._gui_mouse_click)

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    def shutdown(self):
        self._is_shutdown = True

    def run(self):
        board = CalibrationBoard8by6
        # show temporary view while we create the maps
        gui = draw_validation_gui(None, None)
        cv2.imshow(self._window, gui)
        cv2.waitKey(100)
        # compute the region of interest
        roi: RegionOfInterest = RegionOfInterest(origin=board.chessboard_offset, size=board.size)
        # create maps
        mapx, mapy, mask, shape = compute_homography_maps(self._camera, self._H, self._ppm, roi)

        while not self.is_shutdown:
            cframe = self._robot.camera.capture(block=True, timeout=1)

            # no frame?
            if cframe is None:
                continue

            # get frame as uint8 array
            jpg = cframe.as_uint8()

            # decode JPEG
            distorted: BGRImage = self._jpeg.decode(jpg, pixel_format=TJPF_BGR)

            # rectify image
            rectified: BGRImage = self._camera.rectifier.rectify(distorted, interpolation=cv2.INTER_CUBIC)

            # apply homography
            projected: BGRImage = cv2.remap(rectified, mapx, mapy, cv2.INTER_LINEAR)

            # fix missing pixels
            projected = cv2.inpaint(projected, mask, 3, cv2.INPAINT_NS)

            # flip and rotate the image so that it appears as it is seen from the camera
            projected = cv2.flip(projected, 0)
            projected = cv2.rotate(projected, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # resize projected
            _, _, w, h = VALIDATION_GUI_RIGHT_IMAGE_ROI
            projected = cv2.resize(projected, (w, h), cv2.INTER_CUBIC)

            # display GUI
            gui = draw_validation_gui(rectified, projected)
            cv2.imshow(self._window, gui)
            cv2.waitKey(1)

        # destroy window and exit
        cv2.destroyAllWindows()

    def _quit(self):
        self._is_shutdown = True

    def _gui_mouse_click(self, event, x, y, _, __):
        # check if left mouse click
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        # check if we clicked on the button
        bx, by, bw, bh = VALIDATION_GUI_BTN1_ROI
        if bx < x < bx + bw and by < y < by + bh:
            self._quit()


if __name__ == "__main__":
    # run calibration step
    calibration = CameraExtrinsicsCalibration(quiet=False)
    try:
        calibration.run()
    except KeyboardInterrupt:
        exit(0)
    # get homography
    homography = calibration.H

    # run validation step
    validation = CameraExtrinsicsValidation(homography)
    try:
        validation.run()
    except KeyboardInterrupt:
        exit(0)
