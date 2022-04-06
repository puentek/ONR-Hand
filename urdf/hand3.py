# import pybullet as p 
# import pybullet_data 
# datapath = pybullet_data.getDataPath()

# p.connect(p.GUI)
# p.loadURDF("urdf_screencast.urdf")

import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
# useFixedBase = True
planeId = p.loadURDF("urdf_assem4_1.urdf", useFixedBase = True)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
shift = [0, -0.02, 0]
meshScale = [0.1, 0.1, 0.1]

visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="duck.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="duck_vhacd.obj",
                                          collisionFramePosition=shift,
                                          meshScale=meshScale)



startPos = [0,0,10]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("urdf_screencast.urdf",startPos, startOrientation)
# make sure to add ground plane ; want your hand to be horizontal 
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# this runs the simulation 
#this is where i pass the motor commands 
for i in range (10000):
    p.stepSimulation()
    # makes simulation real time 
    time.sleep(1./240.)
    for j in range(10000):
        p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[((-10000 / 2) + i) * meshScale[0] * 2,
                                    (-10000 / 2 + j) * meshScale[1] * 2, 1],
                      useMaximalCoordinates=True)

# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
currentColor = 0

while (1):
  time.sleep(1./240.)


p.disconnect()

# fix urdf file joints are linked and shouldn't be 
# motor commands readl pybullet guide 
#make sure all inertias are correct 


# urdf should have collision properties 
#make sure mesh is collection of convex meshes; 
# can split models or
# vhacd : plug in for blender 2.8 ; takes model that runs convex model in it 
# convex hole need more shophisticated algo; (banana)
#