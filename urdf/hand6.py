import os
import pybullet as p
import time
import pybullet_data
from pybullet_object_models import ycb_objects
import numpy as np
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

#load the plane 
# plane = p.loadURDF("plane.urdf")
p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setRealTimeSimulation(0)
SIM_TIMESTEP = 1
p.setTimeStep(SIM_TIMESTEP)

# set the gravity ; can also have this as fixeD
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

# load urdf model 
planeId = p.loadURDF("urdf_assem4_1.urdf", useFixedBase = True)


# reset joint state:
p.resetJointState = (planeId, 1,-0.5,0)
# p.setJointMotorControlArray(planeId,[0,1],p.POSITION_CONTROL, targetPositions=[0,0], positionGains=[0.01,1],velocityGains=[0.1,0.99])

jointFrictionForces = 0
for joint in range(p.getNumJoints(planeId)):
    p.setJointMotorControl2(planeId,joint,p.VELOCITY_CONTROL, targetVelocity=0.02, force = 2.96)


idff = p.addUserDebugParameter("Test force",-2.96, 2.96,0) 



while True:
    try:
        #apply force to last link:
        pend = [0,1.1,0.1]
        Fend = [p.readUserDebugParameter(idff),0,0]
        p.applyExternalForce(planeId,1,Fend, pend, p.WORLD_FRAME)
        p.addUserDebugLine(pend,np.array(pend)+100*np.array(Fend), lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=2)
        p.stepSimulation()
        time.sleep(SIM_TIMESTEP*1e-3)
    except:
        raise 



# for i in range (10000):
    
#     p.stepSimulation()
#     p.setRealTimeSimulation(1)
#     # makes simulation real time 
#     time.sleep(1./240.)
# #joint control 
#     p.setJointMotorControl(p.TORQUE_CONTROL  )

#     # p.setJointMotorControl(bodyIndex = self.bodies [self.bodyIndex], jointIndex= self.jointIndex, controlMode = p.TORQUE_CONTROL, 
#     # force = torque )

# p.disconnect()


# for joint in range(p.getNumJoints(planeId)):
#     # p.stepSimulation()
#     # p.setRealTimeSimulation(1)
#     # # makes simulation real time 
#     # time.sleep(1./240.)
#     p.setJointMotorControl2(planeId,joint,p.VELOCITY_CONTROL, targetVelocity=50, force = 2.96)

# # p.disconnect()





