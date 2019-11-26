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
from sympy.solvers import solve
from sympy import Symbol
import sympy as sp
import math

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
import vrep

# for random number generation
from random import seed
from random import random

armJoints = [0,0,0,0,0]
wheels = [0,0,0,0]
clientID = 0
MAX_FORCE = 25
tcp_handle = ''

# function take pose of object with respect to body frame of the youbot and returns joint config 
# in degrees and a success value which specifies if one of the thetas could not be calculated
def moveArmPose(end_pose, yaw):

    success = True
    yaw = np.radians(yaw)
    
    print("Yaw in radians: " + str(yaw))
    
    M  = transformation_data.M
    S  = transformation_data.S
    
    # length of links 1,2,3 along x - axis of the body
    L1 = transformation_data.jointOffset[1][0] - transformation_data.jointOffset[0][0]
    L2 = transformation_data.jointOffset[2][0] - transformation_data.jointOffset[1][0]
    L3 = transformation_data.jointOffset[3][0] - transformation_data.jointOffset[2][0]

    #coordinates of the joint 0 (base)
    z_j0 = transformation_data.jointOffset[0][2]
    y_j0 = transformation_data.jointOffset[0][1]
    x_j0 = transformation_data.jointOffset[0][0]
    
    # update the goal to be the point j3 hits forcing the tcp to point in -x axis of body
    # and update z to be with respect to the j0 rather than body
    x_c = end_pose[0] + (transformation_data.tcp_body_offset[0] - transformation_data.jointOffset[3][0])
    y_c = end_pose[1]
    z_c = end_pose[2] - transformation_data.jointOffset[0][2]
    
    # get theta0 by projecting onto the zy = plane
    theta0 = -np.arctan2(y_c,z_c)
    
    # get the distance on the zy plane from joint1 to the center point
    # note that this assumes that theta 1 places all the length of the z offset from j0 to j1 along the path to center
    r = np.sqrt(z_c**2 + y_c**2) - (transformation_data.jointOffset[1][2] - transformation_data.jointOffset[0][2])
    print( np )
    s = x_c - L1
    print("r = " + str(r))
    print("s = " + str(s))
    print("L2 = " + str(L2))
    print("L3 = " + str(L3))
    print("should be between 1 and -1 : " + str((r**2 + s**2 - L2**2 - L3**2)/(2*L2*L3)))
    theta2 = -np.arccos((r**2 + s**2 - L2**2 - L3**2)/(2*L2*L3))
    
    theta1 = -np.arctan2(r,s) - np.arctan2(L3*np.sin(theta2), L2 + L3*np.cos(theta2))

    theta3 = -theta1 -theta2 -np.pi    
    theta4 = yaw - theta0
    
    thetaList = [theta0, theta1,theta2,theta3,theta4]
    """
    dist_x = end_pose[0] - x_j0
    
    dist_body = np.sqrt(end_pose[1]**2 + end_pose[2]**2)
    
    dist_j0 = np.sqrt((end_pose[1] - y_j0)**2 + (end_pose[2] - z_j0)**2)
    
    print("d = " + str(dist_body))
    print("a = " + str(dist_j0))
    
    alpha = np.arctan2(end_pose[1],end_pose[2])
    beta = np.arcsin(dist_body*np.sin(alpha)/dist_j0) + np.pi/2
    
    print("dist j0 = " + str(dist_j0))
    print("dist body = " + str(dist_body))
    print("alpha = " + str(alpha*180/np.pi))
    print("beta = " + str(beta*180/np.pi))
    
    
    theta0 = -(np.pi - beta)
    theta4 = yaw - theta0
    print("theta0 = " + str(theta0))
    print("theta4 = " + str(theta4))
    
    ##### estimate theta 1, and try again
    # must solve for theta1
    theta1 = Symbol('theta1')
    print(solve(sp.acos((dist_x + L3 - L1*sp.cos(theta1))/(-L2)) - sp.asin((dist_j0 - L1*sp.sin(theta1))/L2),theta1))
    
    T_d  = np.array([[           0,            0 ,-1  , end_pose[0]],
                     [ np.cos(yaw), -np.sin(yaw) , 0  , end_pose[1]],
                     [ np.sin(yaw),  np.cos(yaw) , 0  , end_pose[2]],
                     [0,0,0,1]]) 

    guess = transformation_data.front_pose
    
    print("guess thetas in radians: \n" + str(guess))
    # get desired angles from end_pose
    [thetalist, success] = mr.IKinSpace(S, M, T_d, guess, 0.1, 0.1)
    
    
    
    #print(success)
    #print(thetalist)
    #if(success):
        #moveArm(thetalist)
    """
    
    for i in range(len(thetaList)):
        thetaList[i] = thetaList[i]*180/np.pi
        if(math.isnan(thetaList[i])):
            success = False
        
    return success,thetaList

def moveArm(thetaList):
    global clientID
    global armJoints
    time_between_movements = .2
    error = .05

    for i in range(len(thetaList)):
        thetaList[i] = thetaList[i]*np.pi/180
        
    joint_movement_order = [0, 4, 1, 2, 3]
    for i in joint_movement_order:
        [e, curr_theta] = vrep.simxGetJointPosition(clientID, armJoints[i], vrep.simx_opmode_streaming)
        goal_theta = thetaList[i]
        step = (goal_theta - curr_theta) / 10
        for j in range(9):
            vrep.simxSetJointPosition(clientID, armJoints[i], step*j + curr_theta, vrep.simx_opmode_streaming)
            time.sleep(.05)
        vrep.simxSetJointPosition(clientID, armJoints[i], goal_theta, vrep.simx_opmode_streaming)

def getPoseFromJoints(thetas):
    M = transformation_data.M
    S = transformation_data.S
    T_pose = mr.FKinSpace(M,S,thetas)
    return T_pose

def grab():
    global clientID
    sig = vrep.simxSetIntSignal(clientID, "youBotGripperState", 0, vrep.simx_opmode_buffer)
    print(sig)

def release():
    global clientID
    vrep.simxSetIntSignal(clientID, "youBotGripperState", 1, vrep.simx_opmode_buffer)

def moveWheels(fl, fr, bl, br):
    global wheels
    global clientID
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
    S1 = transformation_data.S0
    S2 = transformation_data.S1
    S3 = transformation_data.S2
    S4 = transformation_data.S3
    S5 = transformation_data.S4

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
        T = T.dot(expm(S_brac * theta[i]))

    # finally, multiply by M
    T = T.dot(M)
    return T

# this doesn't work. Tried converting by subtraction
def convertToBodyCoordinatesFromSpaceCoordinates(x, y, z):
    #[0.6336228847503662, -0.0013938546180725098, 0.19836857914924622]
    x_offset = 0.6336228847503662
    y_offset = -0.0013938546180725098
    z_offset = 0.19836857914924622
    return (x + x_offset, y + y_offset, z + z_offset)

'''
    Function communicates with vrep to retrieve data on detected objects, transforms to body coordinates
    and returns yaw and pose
'''
def detectCube(clientID,proxSensor,bodyHandle):
    e,prox_body_p = vrep.simxGetObjectPosition(clientID, proxSensor, bodyHandle, vrep.simx_opmode_streaming)
    #print("prox_body_p = " + str(prox_body_p))
    T_body_sensor = np.array([[0, -1, 0, prox_body_p[0]], 
                              [1, 0, 0,  prox_body_p[1]],
                              [0, 0, 1,  prox_body_p[2]],
                              [0,0,0,1]])

    e,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,proxSensor,vrep.simx_opmode_streaming)
    #print("State: " + str(detectionState))
    #print("Point: " + str(detectedPoint))
    #print("Norm Vector: " + str(detectedSurfaceNormalVector))
    
    # calculate yaw from -45 to 45 degrees
    yaw = 0
    if(detectedSurfaceNormalVector[1] != 0):
        yaw = np.arctan2(detectedSurfaceNormalVector[0], detectedSurfaceNormalVector[2])
        yaw = yaw*180/np.pi
        if(yaw > 45 and yaw < 135):
            yaw -= 90
        elif(yaw < -45 and yaw > -135):
            yaw += 90
        elif(yaw > 135):
            yaw -= 180
        elif(yaw < -135):
            yaw += 180
        #detectedPoint[1] += 0.02
        #detectedPoint[0] += 0.02*np.sin(yaw)
        #detectedPoint[2] += 0.02*np.cos(yaw)
            
    e,detect_pose = vrep.simxGetObjectPosition(clientID, detectedObjectHandle, proxSensor, vrep.simx_opmode_streaming)
    
    pose = np.array([[detect_pose[0]], [detect_pose[1]], [detect_pose[2]], [1]])
    body_pose = np.dot(T_body_sensor,pose)
    cube_pose = [body_pose[0][0], body_pose[1][0], body_pose[2][0]]
    
    return detectionState,yaw,cube_pose 


"""

"""


def main():
    # global variables
    global velocity
    global armJoints
    global wheels
    global MAX_FORCE
    global tcp_handle
    # get transformation data
    M  = transformation_data.M
    S  = transformation_data.S
    T_b_s = transformation_data.T_b_s
    zero_pose = transformation_data.zero_pose
    plate_pose = transformation_data.plate_pose
    front_pose = transformation_data.front_pose

    # VREP stuff
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    if clientID!=-1:
        print ('Connected to remote API server')
        
         # gets handle for TCP - used in printing comparison
        e, tcp_handle = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_oneshot_wait)
        e, bodyHandle = vrep.simxGetObjectHandle(clientID, "youBot", vrep.simx_opmode_blocking)
        e, cubeHandle = vrep.simxGetObjectHandle(clientID, "Rectangle16", vrep.simx_opmode_blocking)
        e, proxSensor = vrep.simxGetObjectHandle(clientID, "Proximity_sensor", vrep.simx_opmode_blocking)
        
        # initialize wheel motors
        e1,wheels[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
        e2,wheels[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        e3,wheels[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
        e4,wheels[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)

        #initialize arm joints
        arm_poses = [0,0,0,0,0]
        for i in range(5):
            # get object handle
            e, armJoints[i] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint' + str(i), vrep.simx_opmode_oneshot_wait)
            # set max force
            vrep.simxSetJointForce(clientID, armJoints[i], MAX_FORCE, vrep.simx_opmode_oneshot_wait)
       
        #TESTING if we can move the arm between two poses
        '''
        while(1):
            moveArm(plate_pose)
            #grab()
            time.sleep(1)
            moveArm(front_pose)
            #release()
            time.sleep(1)
        '''

        #testing body frame conversion. doesn't work
        while True:
            #moveArm(zero_pose)
            #print("zero pose: " + str(getPoseFromJoints(zero_pose)))
            #time.sleep(1)
            
            moveArm(transformation_data.front_pose)
            #print("home pose: "+ str(getPoseFromJoints(transformation_data.front_pose)))
            time.sleep(2)
            
            detect, yaw, cubePose = detectCube(clientID,proxSensor,bodyHandle)
            if(detect):
                e,soln = moveArmPose(cubePose, yaw)
            
                soln_pose = getPoseFromJoints(soln)
            
                #print("Success? " + str(e))
                print("Thetas: " + str(soln))
                print("FK soln: " + str(soln_pose))
                print("cube pose: " + str(cubePose))
                if(e):
                    moveArm(soln)
    
        '''

        # move arm to 0 position
        moveArm(zero_pose)

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
