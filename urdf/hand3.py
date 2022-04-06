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
    viz = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=mesh_file)
    if col_file is None:
        col = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.001, 0.001, 0.001])
    else:
        col = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=col_file)
    if pos is None:
        pos = [0, 0, 0]
    if orientation is None:
        orientation = [0, 0, 0, 1]
    


def load_mesh(mesh_file, col_file=None, mass=0, pos=None, orientation=None):
    viz = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=mesh_file)
    if col_file is None:
        col = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.001, 0.001, 0.001])
    else:
        col = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=col_file)
    if pos is None:
        pos = [0, 0, 0]
    if orientation is None:
        orientation = [0, 0, 0, 1]
    return p.createMultiBody(baseMass=mass, baseVisualShapeIndex=viz, baseCollisionShapeIndex=col,
                              basePosition=pos, baseOrientation=orientation)


# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
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