#!/usr/bin/env python3
import time

from dt_duckiematrix_protocols import Matrix


V0 = 1.0


class WheelsIO:

    def __init__(self):
        # create connection to the matrix engine
        self.matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self.robot = self.matrix.robots.DB21M("map_0/vehicle_0")

    def run(self):
        while not self.is_shutdown:
            self.robot.drive(left=V0, right=V0)
            time.sleep(0.1)

    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False


if __name__ == "__main__":
    node = WheelsIO()
    node.run()
