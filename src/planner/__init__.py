"""The leaders' deliberative layer (phase 5): plan a path on the map, then follow it.

Pure maths, no Webots. Followers never use it.
"""

from .params import PlanParams
from .plan import plan
from .pursuit import Pursuit

__all__ = ["PlanParams", "Pursuit", "plan"]
