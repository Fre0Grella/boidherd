"""Placeholder referee (supervisor): does nothing until phase 1 replaces it."""

from controller import Supervisor

referee = Supervisor()
time_step = int(referee.getBasicTimeStep())
while referee.step(time_step) != -1:
    pass
