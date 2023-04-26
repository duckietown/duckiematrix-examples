#!/usr/bin/env python3
import time

from colorir import HSV

from dt_duckiematrix_protocols import Matrix


class LED:

    def __init__(self):
        # create connection to the matrix engine
        matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self.robot = matrix.robots.DB21M("map_0/vehicle_0")
        # lights
        self.lights = [
            self.robot.lights.light0,
            self.robot.lights.light1,
            self.robot.lights.light3,
            self.robot.lights.light4,
        ]

    def run(self):
        h = 0
        while not self.is_shutdown:
            r, g, b = HSV(h, 1.0, 1.0).rgb()
            with self.robot.lights.atomic():
                for light in self.lights:
                    light.color.r = r
                    light.color.g = g
                    light.color.b = b
            time.sleep(0.01)
            h = (h + 1) % 360

    @property
    def is_shutdown(self):
        return False


if __name__ == "__main__":
    node = LED()
    node.run()
