#!/usr/bin/env python3
import time

from duckietown.sdk.robots.duckiebot import DB21J

PWM_DUTY_CYCLE = -0.25


class WheelsIO:

    def __init__(self):
        # create connection to the matrix engine
        self.robot: DB21J = DB21J("map_0/vehicle_0", simulated=True)

    def run(self):
        self.robot.motors.start()
        while not self.is_shutdown:
            self.robot.motors.set_pwm(left=PWM_DUTY_CYCLE, right=PWM_DUTY_CYCLE)
            time.sleep(0.1)
        self.robot.motors.stop()

    @property
    def is_shutdown(self):
        # TODO: link this to SIGINT
        return False


if __name__ == "__main__":
    node = WheelsIO()
    node.run()
