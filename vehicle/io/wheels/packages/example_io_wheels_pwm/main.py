#!/usr/bin/env python3
import time

from dt_duckiematrix_protocols import Matrix


PWM_DUTY_CYCLE = 1.0


class WheelsIO:

    def __init__(self):
        # create connection to the matrix engine
        self.matrix = Matrix("localhost", auto_commit=True)
        # create connection to the vehicle
        self.robot = self.matrix.robots.DB21M("map_0/vehicle_0")

    def run(self):
        while not self.is_shutdown:
            self.robot.drive_pwm(left=PWM_DUTY_CYCLE, right=PWM_DUTY_CYCLE)
            time.sleep(0.1)

    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False


if __name__ == "__main__":
    node = WheelsIO()
    node.run()
