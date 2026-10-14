"""Placeholder drone controller: stays on the floor until phase 1 replaces it."""

from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())
while robot.step(time_step) != -1:
    pass
