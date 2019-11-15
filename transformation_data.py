import numpy as np


'''

NOTE: THIS ASSUMES (X,Y) ARE (0,0). Z SHOULD BE THE ONLY VARIABLE COORDINATE 
(SINCE 0 POSITION IS STACKED)

'''

# Z of TCP given 
Z_TCP = 0.7289 
# Z of Joint 1
Z0 = 0.1442
# Z of Joint 2
Z1 = 0.2454
# Z of Joint 3
Z2 = 0.4004
# Z of Joint 4
Z3 = 0.5352
# Z of Joint 5
Z4 = 0.6316

y_offset = 0.0357

Z_TCP -= Z0
Z4 -= Z0
Z3 -= Z0
Z2 -= Z0
Z1 -= Z0
Z0 -= Z0

# Create Matrix M
# with respect to joint0, using object youBotArmJoint0.ME_Arm1_m0_sub0_sub0
M = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, y_offset],
                [0, 0, 1, Z_TCP],
                [0, 0, 0, 1]])

# Create S vectors
    # first 3 entries are w: axis of rotation
    # last  3 entries are v: displacement of joint from S frame

# S of Joint 1
w0 = np.array([0, 0, 1])
q0 = np.array([0, y_offset, Z0])
v0 = np.cross(-w0,q0)

S0 = np.array([w0[0], w0[1], w0[2], v0[0], v0[1], v0[2]])

# S of Joint 2
w1 = np.array([0, 1, 0])
q1 = np.array([0, y_offset, Z1])
v1 = np.cross(-w1,q1)

S1 = np.array([w1[0], w1[1], w1[2], v1[0], v1[1], v1[2]])

# S of Joint 3

w2 = np.array([0, 1, 0])
q2 = np.array([0, y_offset, Z2])
v2 = np.cross(-w2,q2)

S2 = np.array([w2[0], w2[1], w2[2], v2[0], v2[1], v2[2]])

# S of Joint 4

w3 = np.array([0, 1, 0])
q3 = np.array([0, y_offset, Z3])
v3 = np.cross(-w3,q3)

S3 = np.array([w3[0], w3[1], w3[2], v3[0], v3[1], v3[2]])


# S of Joint 5

w4 = np.array([0, 0, 1])
q4 = np.array([0, y_offset, Z4])
v4 = np.cross(-w4,q4)

S4 = np.array([w4[0], w4[1], w4[2], v4[0], v4[1], v4[2]])
