import os
import pybullet as p
import time
import pybullet_data
from pybullet_object_models import ycb_objects
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
plane = p.loadURDF("data/plane.urdf")
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
urdfFlags = p.URDF_USE_SELF_COLLISION





