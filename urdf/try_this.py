 p.setJointMotorControl2(pandaUid, 0, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(pandaUid, 1, 
                        p.POSITION_CONTROL,math.pi/4.)
        p.setJointMotorControl2(pandaUid, 2, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(pandaUid, 3, 
                        p.POSITION_CONTROL,-math.pi/2.)
        p.setJointMotorControl2(pandaUid, 4, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(pandaUid, 5, 
                        p.POSITION_CONTROL,3*math.pi/4)
        p.setJointMotorControl2(pandaUid, 6, 
                        p.POSITION_CONTROL,-math.pi/4.)
                        