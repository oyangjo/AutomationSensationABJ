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

# Create Matrix M
M = np.array([
                [1, 0, 0,     x_offset],
                [0, 1, 0,     0],
                [0, 0, 1, Z_TCP],
                [0, 0, 0,     1]])

# Create S vectors
    # first 3 entries are w: axis of rotation
    # last  3 entries are v: displacement of joint from S frame

w1 = np.array([0, 0, 1])
w2 = np.array([0, 1, 0])
w3 = np.array([0, 1, 0])
w4 = np.array([0, 1, 0])
w5 = np.array([0, 0, 1])

q1 = np.array([0, 0, Z1])
q2 = np.array([x_offset, 0, Z2])
q3 = np.array([x_offset, 0, Z3])
q4 = np.array([x_offset, 0, Z4])
q5 = np.array([x_offset, 0, Z5])

v1 = np.cross(q1, w1);
v2 = np.cross(q2, w2);
v3 = np.cross(q3, w3);
v4 = np.cross(q4, w4);
v5 = np.cross(q5, w5);

print(np.transpose(w1))
print(v1)

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