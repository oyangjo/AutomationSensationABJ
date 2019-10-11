# -*- coding: utf-8 -*-
"""
Created on Fri Oct 11 14:11:53 2019

@author: BenSisserman
"""

# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import time
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
    
    wheels = [0,0,0,0]
    e1,wheels[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    e2,wheels[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    e3,wheels[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
    e4,wheels[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
    
    armJoints = [0,0,0,0,0]
    arm_poses = [0,0,0,0,0]
    for i in range(5):
        e, armJoints[i] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint' + str(i), vrep.simx_opmode_oneshot_wait)
        e, arm_poses[i] = vrep.simxGetJointPosition(clientID, armJoints[i], vrep.simx_opmode_streaming)

    e5 = vrep.simxSetJointTargetVelocity(clientID, armJoints[0], -200, vrep.simx_opmode_streaming)

    """
    e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[0], 300, vrep.simx_opmode_streaming)
    e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[1], 300, vrep.simx_opmode_streaming)
    e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[2], 300, vrep.simx_opmode_streaming)
    e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[3], 300, vrep.simx_opmode_streaming)
    """
    time.sleep(5)


    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')




    
