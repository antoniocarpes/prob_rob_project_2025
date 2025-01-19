import math 
import numpy as np

def v2t(v):
    x = v[0]
    y = v[1]
    theta = v[2]
    
    # Initialize a 3x3 matrix of zeros
    T = np.zeros([3, 3])
    
    # Fill the top-left 2x2 submatrix with the rotation matrix
    T[0:2, 0:2] = [[math.cos(theta), -math.sin(theta)], 
                   [math.sin(theta), math.cos(theta)]]
    
    # Set the last column for translation and homogeneous coordinate
    T[0:2, 2] = [x, y]  # Translation in x and y
    T[2, 2] = 1          # Homogeneous coordinate
    
    return T

def t2v(T):
    v = np.zeros(3)
    v[0:2] = T[0:2,2]
    v[2] = math.atan2(T[0,1], T[0,0])
    return v

#print(t2v(np.array([[1,0,12],[0,1,24],[0,0,1]])))
#print(v2t([12,54, math.pi/4]))