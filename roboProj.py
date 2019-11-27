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
import vrep

# for random number generation
from random import seed
from random import random

armJoints = [0,0,0,0,0]
wheels = [0,0,0,0]
clientID = 0
MAX_FORCE = 25
tcp_handle = 0
bodyHandle = 0
def moveArmPose(end_pose):

    M  = transformation_data.M
    S  = transformation_data.S
    
    T_d  = np.array([[0, -0.462, -0.89, end_pose[0]],
                     [1, 0, 0, end_pose[1]],
                     [0, -0.89, 0.46, end_pose[2]],
                     [0,0,0,1]]) 

    #guess = [0, 0, 0, 0, 0]
    # get desired angles from end_pose
    [thetalist, success] = mr.IKinSpace(S, M, T_d, transformation_data.front_pose, 0.1, 0.1)
    
    for i in range(len(thetalist)):
        thetalist[i] = thetalist[i]*180/np.pi
    
    #print(success)
    #print(thetalist)
    if(success):
        moveArm(thetalist)
    return success,thetalist

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

def getPoseFromJoints(thetas):
    M = transformation_data.M
    S = transformation_data.S
    T_pose = mr.FKinSpace(M,S,thetas)
    return T_pose

def grab():
    global clientID
    vrep.simxCallScriptFunction(clientID, "youBot", vrep.sim_scripttype_childscript, "sysCall_test_close", [0], [0], '', '', vrep.simx_opmode_blocking)

def release():
    global clientID
    vrep.simxCallScriptFunction(clientID, "youBot", vrep.sim_scripttype_childscript, "sysCall_test_open", [0], [0], '', '', vrep.simx_opmode_blocking)

def moveWheels(fl, fr, bl, br):
    global wheels
    global clientID

    velocity = 400

    fl *= velocity
    fr *= velocity
    bl *= velocity
    br *= velocity

    #moves the wheels
    e1 = vrep.simxSetJointTargetVelocity(clientID, wheels[0], fl, vrep.simx_opmode_streaming)
    e2 = vrep.simxSetJointTargetVelocity(clientID, wheels[1], fr, vrep.simx_opmode_streaming)
    e3 = vrep.simxSetJointTargetVelocity(clientID, wheels[2], bl, vrep.simx_opmode_streaming)
    e4 = vrep.simxSetJointTargetVelocity(clientID, wheels[3], br, vrep.simx_opmode_streaming)
    return [e1, e2, e3, e4]

def stopWheels():
    moveWheels(0,0,0,0)

def moveToDestination(destination):
    global wheels
    global clientID
    global armJoints
    global tcp_handle
    global bodyHandle

    # Get the current position of the TCP
    tcp_pos = vrep.simxGetObjectPosition(clientID, tcp_handle, -1, simx_opmode_buffer)

    # Get the current position of the robot body (so we know the direction the robot is facing)
    body_pos = vrep.simxGetObjectPosition(clientID, bodyHandle, -1, simx_opmode_buffer)

    distance_to_dest = np.sqrt( (tcp_pos[0] - destination[0])**2 + (tcp_pos[1] - destination[1])**2 )
    while(distance_to_dest <= .5) {
        
        # Check if there's an obsticle in front of us
        # TO DO


        # Get current rotation of the robot body
        body_orientation = vrep.simxGetObjectReotation(clientID, bodyHandle, -1, simx_opmode_buffer)

        # Rotate until the orientation faces the destination
        # TO DO
        
        # Move forward
        moveWheels(1,1,1,1)

        distance_to_dest = np.sqrt( (tcp_pos[0] - destination[0])**2 + (tcp_pos[1] - destination[1])**2 )
    }

    # Arrived at destination
    stopWheels()

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
    global bodyHandle
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
            =release()
            time.sleep(1)
            grab()
            time.sleep(1)
            #moveArm(zero_pose)
            #print("zero pose: " + str(getPoseFromJoints(zero_pose)))
            #time.sleep(1)
            '''
            moveArm(transformation_data.front_pose)
            #print("home pose: "+ str(getPoseFromJoints(transformation_data.front_pose)))
            time.sleep(2)
            
            detect, yaw, cubePose = detectCube(clientID,proxSensor,bodyHandle)
            
            e,soln = moveArmPose(cubePose)
            
            soln_pose = getPoseFromJoints(soln)
            
            print("Success? " + str(e))
            print("Thetas: " + str(soln))
            print("FK soln: " + str(soln_pose))
            print("cube pose: " + str(cubePose))
            '''
        
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
