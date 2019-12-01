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
import matplotlib.pyplot as plt
import cv2
import numpy.linalg as la
import time
from scipy import stats

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
DIST_ERROR = 1
holdingCube = False

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
# function take pose of object with respect to body frame of the youbot and returns joint config 
# in degrees and a success value which specifies if one of the thetas could not be calculated
def findThetas(end_pose, yaw):

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
    theta4 = theta0 - yaw
    
    thetaList = [theta0, theta1,theta2,theta3,theta4]
    
    for i in range(len(thetaList)):
        if(math.isnan(thetaList[i])):
            success = False
        
    return success,thetaList

def grabCube(cubePose, yaw):
    cur_state = transformation_data.EXCAVATE
    
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

    # default joint movement order
    if(len(joint_movement_order) == 0):
        joint_movement_order = [0, 4, 1, 2, 3]

    for i in joint_movement_order:
        [e, curr_theta] = vrep.simxGetJointPosition(clientID, armJoints[i], vrep.simx_opmode_streaming)
        goal_theta = thetaList[i]
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
    global holdingCube
    global clientID
    j, u, n, grabRet, k = vrep.simxCallScriptFunction(clientID, "youBot", vrep.sim_scripttype_childscript, "sysCall_test_close", [0], [0], '', '', vrep.simx_opmode_blocking)
    print(grabRet[0])
    if(float(grabRet[0]) < -0.0):
            holdingCube = True
    time.sleep(1)

def release():
    global holdingCube
    global clientID
    vrep.simxCallScriptFunction(clientID, "youBot", vrep.sim_scripttype_childscript, "sysCall_test_open", [0], [0], '', '', vrep.simx_opmode_blocking)
    holdingCube = False
    time.sleep(0.5)


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
    moveWheels(-1, 1, -1, 1)
def turnRight(theta):
    moveWheels(1, -1, 1, -1)
def moveRight():
    moveWheels(-1, 1, 1, -1)
def moveLeft():
    moveWheels(1, -1, -1, 1)
def moveForward():
    moveWheels(-1, -1, -1, -1)
def moveBackward():
    moveWheels(1, 1, 1, 1)
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
def detectCube():
    global clientID
    global proxSensor
    global bodyHandle
    e,prox_body_p = vrep.simxGetObjectPosition(clientID, proxSensor, bodyHandle, vrep.simx_opmode_oneshot_wait)
    #print("prox_body_p = " + str(prox_body_p))
    T_body_sensor = np.array([[0, -1, 0, prox_body_p[0]], 
                              [1, 0, 0,  prox_body_p[1]],
                              [0, 0, 1,  prox_body_p[2]],
                              [0,0,0,1]])

    e,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,proxSensor,vrep.simx_opmode_oneshot_wait)
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
            
    e,detect_pose = vrep.simxGetObjectPosition(clientID, detectedObjectHandle, proxSensor, vrep.simx_opmode_oneshot_wait)
    
    pose = np.array([[detect_pose[0]], [detect_pose[1]], [detect_pose[2]], [1]])
    body_pose = np.dot(T_body_sensor,pose)
    cube_pose = [body_pose[0][0], body_pose[1][0], body_pose[2][0]]
    
    return detectionState,yaw,cube_pose 

'''
initialize all parameters for blob detection
'''
def blob_search_init():
        params = cv2.SimpleBlobDetector_Params()
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 7
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.2
        # Filter by Inerita
        params.filterByInertia = True
        params.minInertiaRatio = 0.3
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.3

        blob_detector = cv2.SimpleBlobDetector_create(params)
        return blob_detector

'''
Get the distance between vision sensor and cube
Have linear regression with x and 1/y
input: x (float), the pixel size of the detected blob
output: 1/y * alpha (float), distance between vision sensor and cube
'''
def getBlobDist(x):
    alpha = 1.1
    blobSize = np.array([ 10.770329,  9.89123,   8.944272,  8.0622577,  7.28010,    6.32455,   5.4649858, 5.0,        4.472136,  3.6055512, 3.1622777])
    distVsCube = np.array([0.358878,  0.3818426, 0.4411265, 0.49202667, 0.5928447,  0.6524103, 0.7914521, 0.8638230,  0.9806854, 1.1742128, 1.4958543])
    inverse = np.array([1/d for d in distVsCube])                                         #do 1/d becaus that is the relation
    slope, intercept, r_value, p_value, std_err = stats.linregress(blobSize,inverse)      #get slope and intercept
    
    y = intercept + slope * x
    return (1/y) * alpha

'''
Get cube's properties: the distance and center of the cube
use open cv to detect blob's size and certer
Call getBlobDist to convert size to distance of blob
input: clientID, vsHandle
output: blobDist(float), blobCenter(np.array(x, y ,z))
'''
def getCubeProperties(clientID, vsHandle):
    #moveArm(transformation_data.front_pose, [0,3,2,1,4])
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, vsHandle, 0, vrep.simx_opmode_buffer)
    
    if err == vrep.simx_return_ok:  #checking if there is an error
        
        # Reshaping the imgae to the right np array
        img = np.array(image,dtype=np.uint8)
        img.resize([resolution[1],resolution[0],3])   #this image is upsidedown
        img = np.flip(img, 1)
        img = np.flip(img)

        # Define a mask using the lower and upper bounds of the green color
        lower =(150, 200, 0)
        upper = (180, 255, 200)
        mask_image = cv2.inRange(img, lower, upper)
        
        # find centroid of the blobs
        blob_detector = blob_search_init()
        reverse_mask = 255 - mask_image
        keypoints = blob_detector.detect(reverse_mask)
        
        # find coordinate and size of the blob. The criteria is only one blob
        blobCenter, blobSize = [], 0
        if len(keypoints) == 1:
            #print('Detecting one blob!')
            blobCenter = np.array(keypoints[0].pt)
            blobSize = keypoints[0].size

        blobDist = getBlobDist(blobSize)
        
        # getting position of the vision sensor/cube and the distance between the two
        e, vsPose = vrep.simxGetObjectPosition(clientID, vsHandle, -1, vrep.simx_opmode_streaming)
        #e, cubePose = vrep.simxGetObjectPosition(clientID, cubeHandle, -1, vrep.simx_opmode_streaming)
        #disVsCube = np.sqrt(np.sum((np.array(vsPose) - np.array(cubePose))**2, axis = 0))
        #print(disVsCube)
        '''
        # Draw detected blobs as red circles.
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # display images
        cv2.imshow("Mask Window", mask_image)
        cv2.imshow("Blob Centroids", im_with_keypoints)
        cv2.imshow('Image',img)
        '''
        
        return blobDist, blobCenter
    elif err == vrep.simx_return_novalue_flag:
        print("no image yet")
        pass
    else:
      print(err)
'''
Deposits cube to the bin.
Assumes that we are already holding a cube
'''
def returnCube():
    bin = transformation_data.bin_coords
    home = transformation_data.home_coords
    back_arm = transformation_data.plate_pose
    front_arm = transformation_data.front_pose

    #Move to the bin
    moveArm(back_arm, [])
    moveToDestination(bin)

    #Deposit the cube into the bin
    moveArm(front_arm, [])
    release()
    time.sleep(0.5)

    #move away from the bin
    moveArm(back_arm, [])
    moveBackward()
    time.sleep(1)
    stopWheels()


def main():
    # global variables
    global velocity
    global armJoints
    global wheels
    global MAX_FORCE
    global tcp_handle
    global bodyHandle
    global proxSensor
    global holdingCube
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
        #e, visionSensorHandle = vrep.simxGetObjectHandle(clientID, "Vision_sensor", vrep.simx_opmode_blocking)
        
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
        #IMAGE PROCESSING
        res, vsHandle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
        res, bsHandle = vrep.simxGetObjectHandle(clientID, 'Has_Block_sensor', vrep.simx_opmode_oneshot_wait)
        res, cubeHandle = vrep.simxGetObjectHandle(clientID, 'Rectangle14', vrep.simx_opmode_oneshot_wait)
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, vsHandle, 0, vrep.simx_opmode_streaming)
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, bsHandle, 0, vrep.simx_opmode_streaming)
        
        #TESTING if we can move the arm between two poses
        '''
        DO NOT DELETE
        NEEDED TO MAKE ROBOT GET CORRECT POSITION AT START (wait until buffer is cleared)
        '''
        print("Initializing robot...\n\n")
        time.sleep(2)


        stopWheels()
        moveArm(transformation_data.plate_pose,[])
        release()
        ### INSERT YOUR CODE HERE ###

        # MEANS NO CUBE -0.046000331640244
        '''
        while True:
            grab()
            release()
        '''

        continue_running = True
        while continue_running: # This will happen as long as the robot is alive
            print("Start of main loop")
            start_time = time.time()
            # STATE 1
            #Search for a cube
            dist, blob_center = getCubeProperties(clientID, vsHandle)
            while(dist < 0):
                turnRight(0)
                #print(dist)
                dist, blob_center = getCubeProperties(clientID, vsHandle)
                print(time.time() - start_time)
                if(time.time() - start_time > 15): #If we've been searching for 15 seconds
                    #Give up
                    continue_running = False
                    break
            stopWheels()

            if(not continue_running):
                print("Giving up...")
                break

            # STATE 2
            #Navigate to cube
            print("Navigating to cube")
            while(len(blob_center) != 0 and blob_center[0] > 140): # MOVE RIGHT UNTIL BLOCK IS CENTERED
                moveRight()
                dist, blob_center = getCubeProperties(clientID, vsHandle)
            while(len(blob_center) != 0 and blob_center[0] < 110): # MOVE LEFT UNTIL BLOCK IS CENTERED
                moveLeft()
                dist, blob_center = getCubeProperties(clientID, vsHandle)
            while(len(blob_center) != 0 and blob_center[1] < 90): # MOVE FORWARD UNTIL BLOCK IS WITHIN REACH
                moveForward()
                dist, blob_center = getCubeProperties(clientID, vsHandle)
            stopWheels()

            # STATE 3
            #Grab cube
            #while True:

            print("Grabbing cube")
            detectionState, yaw, cube_pose  = detectCube()
            grabCube_success = grabCube(cube_pose, yaw)
            
            # Check if we have cube
            if(grabCube_success):
                time.sleep(1)
                dist, blob_center = getCubeProperties(clientID, bsHandle)
                if(len(blob_center) == 0):
                    grabCube_success = False
            print("Successfully grabbed cube: " + str(grabCube_success))
            #STATE 4
            #Move to destination
            if(grabCube_success):
                returnCube()
            release()
        time.sleep(5)
            
            
        
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')




if __name__ == '__main__':
    main()
