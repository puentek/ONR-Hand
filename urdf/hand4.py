import os

from zmq import MAX_SOCKETS_DFLT
import pybullet as p
import time
import pybullet_data
from pybullet_object_models import ycb_objects
import numpy as np
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

#load the plane 
#plane = p.loadURDF("plane.urdf")
#TIME STEP
SIM_TIMESTEP = 1
# set the gravity ; can also have this as fixeD
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setTimeStep(SIM_TIMESTEP)

# load urdf model 
planeId = p.loadURDF("urdf_assem4_1.urdf", useFixedBase = True)


# reset joint state:
p.resetJointState = (planeId, 1,-0.5,0)
p.setJointMotorControlArray(planeId,[0,1],p.POSITION_CONTROL, targetPosition=[0,0], positionGains=[0.01,1],velocityGains=[0.1,0.99])

idff = p.addUserDegugParameter("Test force",-0.01, 0.01,0) 

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








