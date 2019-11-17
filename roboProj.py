# -*- coding: utf-8 -*-
"""
Created on Fri Oct 11 14:11:53 2019

@author: BenSisserman


SOURCE: https://github.com/NxRLab/ModernRobotics

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
import modern_robotics as mr

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



from scipy.linalg import expm

# transformation_data.py holds all forward kinematic variables
import transformation_data

# for random number generation
from random import seed
from random import random

armJoints = [0,0,0,0,0]
wheels = [0,0,0,0]
clientID = 0
MAX_FORCE = 25

def moveArmPose(end_pose):

    M  = transformation_data.M
    S  = transformation_data.S

    guess = [0, 0, 0, 0, 0]
    # get desired angles from end_pose
    [thetalist, success] = mr.IKinSpace(S, M, end_pose, guess, 0.1, 0.1)

    moveArm(thetalist)

def moveArm(thetalist):
    global clientID
    global armJoints
    time_between_movements = .2
    error = .05

    joint_movement_order = [0, 4, 1, 2, 3]
    for i in joint_movement_order:
        [e, curr_theta] = vrep.simxGetJointPosition(clientID, armJoints[i], vrep.simx_opmode_streaming)
        goal_theta = thetalist[i]
        step = (goal_theta - curr_theta) / 10
        for j in range(9):
            vrep.simxSetJointPosition(clientID, armJoints[i], step*j + curr_theta, vrep.simx_opmode_streaming)
            time.sleep(.01)
        vrep.simxSetJointPosition(clientID, armJoints[i], goal_theta, vrep.simx_opmode_streaming)

def moveWheels(fl, fr, bl, br):
    global wheels
    #moves the wheels
    e1 = vrep.simxSetJointTargetVelocity(clientID, wheels[0], fl, vrep.simx_opmode_streaming)
    e2 = vrep.simxSetJointTargetVelocity(clientID, wheels[1], fr, vrep.simx_opmode_streaming)
    e3 = vrep.simxSetJointTargetVelocity(clientID, wheels[2], bl, vrep.simx_opmode_streaming)
    e4 = vrep.simxSetJointTargetVelocity(clientID, wheels[3], br, vrep.simx_opmode_streaming)
    return [e1, e2, e3, e4]

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
    global velocity
    global armJoints
    global wheels
    global MAX_FORCE

    M  = transformation_data.M
    S  = transformation_data.S
    zero_pose = transformation_data.zero_pose
    plate_pose = transformation_data.plate_pose
    front_pose = transformation_data.front_pose




    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    if clientID!=-1:
        print ('Connected to remote API server')
        

        # initialize wheel motors
        e1,wheels[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
        e2,wheels[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        e3,wheels[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
        e4,wheels[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)

        #initialize arm motors
        arm_poses = [0,0,0,0,0]
        
        for i in range(5):
            e, armJoints[i] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint' + str(i), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointForce(clientID, armJoints[i], MAX_FORCE, vrep.simx_opmode_oneshot_wait)
        
        while(1):
            moveArm(plate_pose)
            time.sleep(2)
            moveArm(front_pose)
            time.sleep(2)
            #print(str(i) + "th Joint: " + str(curr_theta))
        '''


        # move arm to 0 position
        moveArm(zero_pose)

        # gets handle for TCP - used in printing comparison
        e, tcp_handle = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_oneshot_wait)


        # init theta arr
        theta = [0, 0, 0, 0, 0]
        for j in range (6):
            if(j == 1):
                theta = [np.pi/4, 0, 0, 0, 0]
            elif(j == 2):
                theta = [0, np.pi/4, 0, 0, 0]
            elif(j == 3):
                theta = [0, 0, np.pi/4, 0, 0]
            elif(j == 4):
                theta = [0, 0, 0, np.pi/4, 0]
            elif(j == 5):
                theta = [0, 0, 0, 0, np.pi/4]

            # move arm to some location
            moveArm(theta)

            # get forward kinematics
            T = getT(theta)

            # print forward kinematics
            print("\nCalculated Forward Kinematics:")
            #print(T)
            print(mr.FKinSpace(M, S, theta))
            print("\n")

            # get inverse kinematics
            [thetalist, success] = mr.IKinSpace(S, M, T, theta, 0.1, 0.1)

            # print inverse kinematics
            print("\nCalculated Inverse Kinematics:\n")
            print(str(thetalist))
            print("\n")
            print("\nReal Thetas:\n" + str(theta)+ "\n")

            #wait until keypress
            input("Press Enter to continue...")
        

        time.sleep(1)
        '''
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')




if __name__ == '__main__':
    main()
