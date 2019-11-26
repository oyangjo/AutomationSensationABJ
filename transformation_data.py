import numpy as np


'''

NOTE: THIS ASSUMES (X,Y) ARE (0,0). Z SHOULD BE THE ONLY VARIABLE COORDINATE 
(SINCE 0 POSITION IS STACKED)

'''

# Z of TCP given 
Z_TCP = 0.7289 
# Z of Joint 1
Z1 = 0.1442
# Z of Joint 2
Z2 = 0.2454
# Z of Joint 3
Z3 = 0.4004
# Z of Joint 4
Z4 = 0.5352
# Z of Joint 5
Z5 = 0.6316

x_offset = -0.0357


Z_TCP -= Z1
Z5 -= Z1
Z4 -= Z1
Z3 -= Z1
Z2 -= Z1
Z1 -= Z1


tcp_body_offset = [0.633, 0, 0.200]
jointOffset = [[0.049, 0, 0.166],
               [0.150, 0, 0.200],
               [0.305, 0, 0.200],
               [0.440, 0, 0.200],
               [0.536, 0, 0.200]
               ]

# Create Matrix M
# with respect to joint0, using object youBotArmJoint0.ME_Arm1_m0_sub0_sub0
M = np.array([
                [0, 0, 1, tcp_body_offset[0]],
                [1, 0, 0, tcp_body_offset[1]],
                [0, 1, 0, tcp_body_offset[2]],
                [0, 0, 0, 1]])

# Create S vectors
    # first 3 entries are w: axis of rotation
    # last  3 entries are v: displacement of joint from S frame

w1 = np.array([1, 0, 0])
w2 = np.array([0, 1, 0])
w3 = np.array([0, 1, 0])
w4 = np.array([0, 1, 0])
w5 = np.array([1, 0, 0])

q1 = np.array(jointOffset[0])
q2 = np.array(jointOffset[1])
q3 = np.array(jointOffset[2])
q4 = np.array(jointOffset[3])
q5 = np.array(jointOffset[4])

v1 = np.cross(q1, w1);
v2 = np.cross(q2, w2);
v3 = np.cross(q3, w3);
v4 = np.cross(q4, w4);
v5 = np.cross(q5, w5);

# S of Joint 1
S1 = np.array([ 
    [w1[0]], 
    [w1[1]], 
    [w1[2]], 
    [v1[0]], 
    [v1[1]], 
    [v1[2]]
    ])
# S of Joint 2
S2 = np.array([ 
    [w2[0]], 
    [w2[1]], 
    [w2[2]], 
    [v2[0]], 
    [v2[1]], 
    [v2[2]]
    ])
# S of Joint 3
S3 = np.array([ 
    [w3[0]], 
    [w3[1]], 
    [w3[2]], 
    [v3[0]], 
    [v3[1]], 
    [v3[2]]
    ])
# S of Joint 4
S4 = np.array([ 
    [w4[0]], 
    [w4[1]], 
    [w4[2]], 
    [v4[0]], 
    [v4[1]], 
    [v4[2]]
    ])
# S of Joint 5
S5 = np.array([ 
    [w5[0]], 
    [w5[1]], 
    [w5[2]], 
    [v5[0]], 
    [v5[1]], 
    [v5[2]]
    ])

# S matrix
S = np.concatenate((S1, S2, S3, S4, S5), axis=1)

# Transformation matrix (from base coordinates to body frame coordinates):
#end-effector coordinates in robot body frame coordinates:
T_e_b = np.array([
        [1,     0,  0,   0.6336122155189514   ],
        [0,     1,  0,   -0.001332700252532959],
        [0,     0,  1,   0.19687017798423767  ],
        [0,     0,  0,   1                    ]])

#end-effector coordinates in space coordinates:
T_e_s = np.array([
    [ 1,      0,      0,     -0.0357],
    [ 0,      1,      0,      0.    ],
    [ 0,      0,      1,      0.5847],
    [ 0,      0,      0,      1.    ]])

# we want space coordinates relative to base coordinates,
# so T_b_s = T_b_e * T_e_s = T_e_b^-1 * T_e_s
T_b_s = np.linalg.inv(T_e_b) * T_e_s



zero_pose = [0, 0, 0, 0, 0]

plate_pose = np.radians([0, 30.9, 52.4, 72.7, 0])


front_pose = np.radians([0, -30.4, -45.6, -76.5, 0])
