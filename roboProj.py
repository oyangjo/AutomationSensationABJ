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
tcp_handle = 0
bodyHandle = 0
TURN_ERROR = 5 * math.pi/180
DIST_ERROR = 0.05

def moveArmPose(end_pose):
    cur_state = 0

# function take pose of object with respect to body frame of the youbot and returns joint config 
# in degrees and a success value which specifies if one of the thetas could not be calculated
def moveArmPose(end_pose, yaw):
    success = True

    # length of links 1,2,3 along x - axis of the body
    L1 = transformation_data.jointOffset[1][0] - transformation_data.jointOffset[0][0]
    L2 = transformation_data.jointOffset[2][0] - transformation_data.jointOffset[1][0]
    L3 = transformation_data.jointOffset[3][0] - transformation_data.jointOffset[2][0]

    # update the goal to be the point j3 hits forcing the tcp to point in -x axis of body
    # and update z to be with respect to the j0 rather than body
    x_c = end_pose[0] + (transformation_data.tcp_body_offset[0] - transformation_data.jointOffset[3][0]) - transformation_data.jointOffset[0][0]/2
    y_c = end_pose[1]
    z_c = end_pose[2] - transformation_data.jointOffset[0][2]
    
    # get theta0 by projecting onto the zy = plane
    theta0 = -np.arctan2(y_c,z_c)
    
    # get the distance on the zy plane from joint1 to the center point
    # note that this assumes that theta 1 places all the length of the z offset from j0 to j1 along the path to center
    r = np.sqrt(z_c**2 + y_c**2) - (transformation_data.jointOffset[1][2] - transformation_data.jointOffset[0][2])
    s = x_c - L1 

    theta2 = -np.arccos((r**2 + s**2 - L2**2 - L3**2)/(2*L2*L3))
    theta1 = -np.arctan2(r,s) - np.arctan2(L3*np.sin(theta2), L2 + L3*np.cos(theta2))
    theta3 = -theta1 -theta2 -np.pi    
    theta4 = 0
    
    thetaList = [theta0, theta1,theta2,theta3,theta4]
    
    for i in range(len(thetaList)):
        if(math.isnan(thetaList[i])):
            success = False
        
    return success,thetaList

def grabCube(cubePose, yaw):
    global cur_state
    
    if(cur_state != transformation_data.EXCAVATE):
        print("ERROR: not in excavation mode.")
        return False
    
    success, thetaList = findThetas(cubePose,yaw)
    if(success):
        moveArm(thetaList, [0,3,2,1,4])
        grab()
        moveArm(transformation_data.front_pose, [4,3,2,1,0])
        moveArm(transformation_data.plate_pose, [0,3,2,1,4])
        cur_state = transformation_data.DRIVE_LOAD
        return True
    else:
        print("Cube out of reach.")
        return False
        
def dropCube():
    global cur_state
    if(cur_state != transformation_data.DEPOSIT):
        print("Error: Not in Deposit state. Status of cube unknown.")
        return False
    
    moveArm(transformation_data.front_pose,[0,3,2,1,4])
    release()
    cur_state = transformation_data.DONE
    return True

def moveArm(thetaList, joint_movement_order):
    global clientID
    global armJoints
    time_between_movements = .2
    error = .05

    for i in joint_movement_order:
        [e, curr_theta] = vrep.simxGetJointPosition(clientID, armJoints[i], vrep.simx_opmode_streaming)
        goal_theta = thetalist[i]
        step = (goal_theta - curr_theta) / 10
        for j in range(9):
            vrep.simxSetJointPosition(clientID, armJoints[i], step*j + curr_theta, vrep.simx_opmode_streaming)
            time.sleep(.01)
        vrep.simxSetJointPosition(clientID, armJoints[i], goal_theta, vrep.simx_opmode_streaming)
        time.sleep(0.05)

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

    velocity = 4

    fl *= velocity
    fr *= velocity
    bl *= velocity
    br *= velocity
    #moves the wheels
    e1 = vrep.simxSetJointTargetVelocity(clientID, wheels[0], fl, vrep.simx_opmode_oneshot)
    e2 = vrep.simxSetJointTargetVelocity(clientID, wheels[1], bl, vrep.simx_opmode_oneshot)
    e3 = vrep.simxSetJointTargetVelocity(clientID, wheels[2], br, vrep.simx_opmode_oneshot)
    e4 = vrep.simxSetJointTargetVelocity(clientID, wheels[3], fr, vrep.simx_opmode_oneshot)
    return [e1, e2, e3, e4]
def turnLeft(theta):
    #print("Turning left")
    moveWheels(-1, 1, -1, 1)
def turnRight(theta):
    #print("Turning right")
    moveWheels(1, -1, 1, -1)
def stopWheels():
    moveWheels(0,0,0,0)


def moveToDestination(destination):
    global wheels
    global clientID
    global armJoints
    global bodyHandle
    global DIST_ERROR

    # Get the current position of the robot body (so we know the direction the robot is facing)
    e, body_pos = vrep.simxGetObjectPosition(clientID, bodyHandle, -1, vrep.simx_opmode_buffer)
    distance_to_dest = np.sqrt( (body_pos[0] - destination[0])**2 + (body_pos[1] - destination[1])**2 )
    print("Distance to goal: " + str(distance_to_dest))

    turnToGoal(destination)
    moveWheels(-1,-1,-1,-1)
    angleCorrectionTick = 0
    while(distance_to_dest > DIST_ERROR):
        print("Distance to goal: " + str(distance_to_dest))
        # CHECK IF OBSTICLE
            # to do
        e, body_pos = vrep.simxGetObjectPosition(clientID, bodyHandle, -1, vrep.simx_opmode_buffer)
        distance_to_dest = np.sqrt( (body_pos[0] - destination[0])**2 + (body_pos[1] - destination[1])**2 )
        time.sleep(.5)
        if(angleCorrectionTick == 5): # correct angle every 2.5 seconds
            turnToGoal(destination)
            moveWheels(-1,-1,-1,-1) # Turn to goal turns off the wheels. Turn them back on
            angleCorrectionTick = 0
        angleCorrectionTick += 1

    # Arrived at destination
    stopWheels()

    print("Arrived!")


# Turns towards the point given in the world frame
def turnToGoal(goal):
    global wheels
    global clientID
    global bodyHandle
    global TURN_ERROR
    e, curr_pos = vrep.simxGetObjectPosition(clientID, bodyHandle, -1, vrep.simx_opmode_buffer)
    print(curr_pos)
    #Given by 90 - arctan(∆Y/∆X)
    goal_rot = math.atan2(curr_pos[1] - goal[1] , curr_pos[0] - goal[0])
    curr_rot = currentRotationAngle()
    #print("Must get orientation to " + str(goal_rot*180/math.pi) + "º. Current orientation is " + str(curr_rot*180/math.pi) + "º.\n")
    # Turn left to increase current angle. Turn right to decrease current angle

    # Calculates most efficient way to turn:
    dir = 1 # Assume turning right
    if(goal_rot < curr_rot):
        dir *= -1
    if(abs(goal_rot - curr_rot) >= math.pi):
        dir *= -1

    if (dir == 1):
        turnRight(0)
    else:
        turnLeft(0)
    while (abs(goal_rot - currentRotationAngle()) > TURN_ERROR):
        time.sleep(.05)
        #print(str(goal_rot) + " " + str(currentRotationAngle()) )
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
def py_ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = np.linalg.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang) * 180 / np.pi
# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R
def rotationMatrixToEulerAngles(R) :
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])
# Returns the rotation matrix of the robot in terms of world coordinates
# given an angle about the world x axis
def getRobotRotationMatrixFromAngle(theta):

    return np.array([
        [0,                 0,                  1],
        [-math.sin(theta),  -math.cos(theta),   0],
        [math.cos(theta),   -math.sin(theta),   0]
        ])
# Returns the angle of rotation of the robot in about the x axis of the world frame
def currentRotationAngle():
    global clientID
    global bodyHandle

    e, body_orientation = vrep.simxGetObjectOrientation(clientID, bodyHandle, -1, vrep.simx_opmode_streaming)
    R = eulerAnglesToRotationMatrix(body_orientation)
    return -math.atan2(  -R[1,0], R[2,0] )
# Just used for testing. Here for safekeeping
def setRotation():
    global clientID, bodyHandle
    new_rot = getRobotRotationMatrixFromAngle(i)
    angles = rotationMatrixToEulerAngles(new_rot)
    vrep.simxSetObjectOrientation(clientID, bodyHandle, -1, angles, vrep.simx_opmode_oneshot)
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
        # MAKE A CALL WITH simx_opmode_streaming TO INIT DATA AQUISITION
        vrep.simxGetObjectPosition(clientID, bodyHandle, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(clientID, bodyHandle, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(clientID, bodyHandle, -1, vrep.simx_opmode_buffer)
        vrep.simxGetObjectOrientation(clientID, bodyHandle, -1, vrep.simx_opmode_buffer)


        '''
        DO NOT DELETE
        NEEDED TO MAKE ROBOT GET CORRECT POSITION AT START (wait until buffer is cleared)
        '''
        print("Initializing robot...\n\n")
        time.sleep(2)




        # TESTING
        moveToDestination([1.4, -0.97, 0.5])
        stopWheels()
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')




if __name__ == '__main__':
    main()
