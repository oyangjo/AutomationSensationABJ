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
import numpy as np
from scipy.linalg import expm

# transformation_data.py holds all forward kinematic variables
import transformation_data

# for random number generation
from random import seed
from random import random

'''
function getT(theta):
    Calculates the transformation matrix of the end-effector frame using forward kinematics
    INPUT: theta- list of angles that the joints are set to
    RETURNS: transformation matrix, whose 4th column holds XYZ coordinates of end-effector
'''
def getT(theta):
    # get data from transformation_data.py
    M  = transformation_data.M
    S1 = transformation_data.S1
    S2 = transformation_data.S2
    S3 = transformation_data.S3
    S4 = transformation_data.S4
    S5 = transformation_data.S5

    # print original frame
    #print(M)
    print("\n")

    # Calculate matrix exponential
    S = [S1, S2, S3, S4, S5]
    T = np.eye(4)
    for i in range (5):
        w1 = S[i][0]
        w2 = S[i][1]
        w3 = S[i][2]
        v1 = S[i][3]
        v2 = S[i][4]
        v3 = S[i][5]
        # S contains skew-symmetric w matrix + v
        S_brac = np.array([
            [0,     -w3,    w2,     v1],
            [w3,    0,      -w1,    v2],
            [-w2,   w1,     0,      v3],
            [0,     0,      0,      0]])
        # chain the matrix multiplication
        T = T.dot( expm(S_brac * theta[i]) )

    # finally, multiply by M
    T = T.dot(M)
    return T

"""
    
"""


def main():
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
        

        # initialize wheel motors
        wheels = [0,0,0,0]
        e1,wheels[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
        e2,wheels[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        e3,wheels[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
        e4,wheels[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)

        #initialize arm motors
        armJoints = [0,0,0,0,0]
        arm_poses = [0,0,0,0,0]
        for i in range(5):
            e, armJoints[i] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint' + str(i), vrep.simx_opmode_oneshot_wait)

        
        '''
        #moves the wheels
        e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[0], 300, vrep.simx_opmode_streaming)
        e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[1], 300, vrep.simx_opmode_streaming)
        e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[2], 300, vrep.simx_opmode_streaming)
        e5 = vrep.simxSetJointTargetVelocity(clientID, wheels[3], 300, vrep.simx_opmode_streaming)
        '''


        '''
        TO DO:

        -CALCULATE INVERSE KINEMATIC MATRIX
        -WRITE FUNCTION TO CALCULATE MATRIX EXPONENTIAL
        -IT WILL RETURN X,Y COORDINATES GIVEN THETA
        -PRINT THOSE COORDINATES 

        -INVERSE KINEMATICS WILL SUIT OUR PROJECT BETTER
        -BECAUSE WE ARE TRYING TO LOCATE A BLOCK AND 
        -PICK IT UP. WE NEED ANGLE MEASUREMENTS
        -GIVEN COORDINATES, THE EXACT OPPOSITE OF 
        -WHAT WE'RE DOING NOW

        -STUFF TO RESEARCH:
        -WAYS TO LOCALIZE ROBOT
        -DETECT OBJECTS (using camera)

        '''
        # gets handle for TCP - used in printing comparison
        e, tcp_handle = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_oneshot_wait)

        # seed the random number generator
        seed(2)
        # init theta arr
        theta = [0, np.pi/4, 0, 0, 0]

        # loop forever (to test forward kinematic calculations)
        while True:
            # move arm to zero location
            for i in range (5):
                vrep.simxSetJointPosition(clientID, armJoints[i], theta[i], vrep.simx_opmode_streaming)
        
            
            # generate random angles
            #for i in range (5):
                #theta[i] = random()

            # move arm to some location
            for i in range (5):
                vrep.simxSetJointPosition(clientID, armJoints[i], theta[i], vrep.simx_opmode_streaming)

            # get transformation matrix
            T = getT(theta)

            # print transformation matrix
            print("\nTransformation Coordinates:")
            print(T)
            
            print("\nActual Coordinates:")
            #print frame 1 with respect to frame 0
            print(vrep.simxGetObjectPosition(clientID, tcp_handle, armJoints[0], vrep.simx_opmode_streaming))


            # wait 10 seconds
            time.sleep(5)

        # will never happen
        time.sleep(5)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')






if __name__ == '__main__':
    main()
