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

Z_TCP -= Z1
Z5 -= Z1
Z4 -= Z1
Z3 -= Z1
Z2 -= Z1
Z1 -= Z1

# Create Matrix M
M = np.array([
                [1, 0, 0,     0],
                [0, 1, 0,     0],
                [0, 0, 1, Z_TCP],
                [0, 0, 0,     1]])

# Create S vectors
    # first 3 entries are w: axis of rotation
    # last  3 entries are v: displacement of joint from S frame

# S of Joint 1
S1 = np.array([
                [0],
                [0],
                [1],
                [0],
                [0],
                [Z1]])

# S of Joint 2
S2 = np.array([
                [0],
                [1],
                [0],
                [0],
                [0],
                [Z2]])

# S of Joint 3
S3 = np.array([
                [0],
                [1],
                [0],
                [0],
                [0],
                [Z3]])

# S of Joint 4
S4 = np.array([
                [0],
                [1],
                [0],
                [0],
                [0],
                [Z4]])

# S of Joint 5
S5 = np.array([
                [0],
                [0],
                [1],
                [0],
                [0],
                [Z5]])