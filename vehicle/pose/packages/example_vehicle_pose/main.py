#!/usr/bin/env python3
import time

from dt_duckiematrix_protocols import Matrix


class VehiclePose:

    def __init__(self):
        # create connection to the matrix engine
        self.matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self.robot = self.matrix.robots.DB21M("map_0/vehicle_0", raw_pose=True)

    def run(self):
        while True:
            self.robot.pose.x = 0.2
            self.robot.pose.relative_to = "map_0"
            print("pose", self.robot.pose)
            print("layer", self.matrix.layers.get("frames", "map_0/vehicle_0"))
            time.sleep(1)

        self.robot.velocity.x = 0
        self.robot.velocity.y = 0
        self.robot.velocity.z = 0


    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False


if __name__ == "__main__":
    node = VehiclePose()
    node.run()
