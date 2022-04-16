# import pybullet as p 
# import pybullet_data 
# datapath = pybullet_data.getDataPath()

# p.connect(p.GUI)
# p.loadURDF("urdf_screencast.urdf")

import os
import pybullet as p
import time
import pybullet_data
from pybullet_object_models import ycb_objects
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
# useFixedBase = True
planeId = p.loadURDF("urdf_assem4_1.urdf", useFixedBase = True)
# bunnyId = p.loadSoftBody("bunny.obj")
# useRealTimeSimulation = 1 



startPos = [0,0,10]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# add object in the simulator 
# boxId = p.loadURDF("cube.urdf", [0,1,0],useFixedBase= True)
# boxId = p.loadURDF("urdf_screencast.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
flags = p.URDF_USE_INERTIA_FROM_FILE
obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbBanana', "model.urdf"),[-0.13, -0.13, 0.2], useFixedBase= True)

# this runs the simulation 
#this is where i pass the motor commands 
for i in range (10000):

    p.stepSimulation()
    p.setRealTimeSimulation(1)
    # makes simulation real time 
    time.sleep(1./240.)

# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)

p.disconnect()

# fix urdf file joints are linked and shouldn't be 
# motor commands readl pybullet guide 
#make sure all inertias are correct 


# urdf should have collision properties 
#make sure mesh is collection of convex meshes; 
# can split models or
# vhacd : plug in for blecander 2.8 ; takes model that runs convex model in it 
# convex hole need more shophisticated algo; (banana)
#