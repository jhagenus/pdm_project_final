from MotionPlanningEnv.sphereObstacle import SphereObstacle

import os

obst1Dict = {
    "type": "sphere",
    'movable': True,
    "geometry": {"position": [1.0, 1.0, 0.5], "radius": 0.5},
}
sphereObst1 = SphereObstacle(name="simpleSphere", content_dict=obst1Dict)

