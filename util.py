import numpy

def makeTransformMat(rot, pt):
    ret = numpy.zeros((4, 4))
    for i in range(0, 3):
        for j in range(0, 3):
            ret[i,j] = rot[i,j]
        ret[i,3] = pt[i]
        ret[3,i] = 0
    ret[3,3] = 1
    return ret

def unpackTransformMat(m):
    return (numpy.matrix(m[0:3,0:3]), numpy.matrix(m[0:3, 3]))

def hat3(v):
    return numpy.matrix([[0,-v[2],v[1]],
                         [v[2],0,-v[0]],
                         [-v[1],v[0],0]])
def hat6(v):
    return numpy.matrix([0,-v[5],v[4],v[0]],
                        [v[5],0,-v[3],v[1]],
                        [-v[4],v[3],0,v[2]],
                        [0,0,0,0])
def wedge3(vhat):
    return numpy.array([vhat[2,1],vhat[0,2],vhat[1,0]])

def wedge6(vhat):
    return numpy.array(
        [vhat[0,3], vhat[1,3], vhat[2,3],
         vhat[2,1], vhat[0,2], vhat[1,0]])

# n in the below comments is the number of joints
# J is the jacobian matrix (6 x n)
# T is a transformation from the base frame representing current pos/ori (6 x n)
# target is a transformation from the base frame (pos/ori) (6 x n)
def resolvedRates(J, T, target, speed):
    pos = numpy.matrix([]);
    
    # need to calc trajectory; straight line? how to avoid singularities?
    # how to later add in collision detection?
    return qdot

def quat2Rot(qx,qy,qz,qw):
    return numpy.matrix([1-2*(qy**2)-2*(qz**2), 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx2-2*(qz**2), 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*(qx**2)-2*(qy**2)])

def transformation(R,p):
    T = numpy.c_[R, p]
    return T = numpy.r_[T,[[0, 0, 0, 1]]]

def dcm2angle(C, output_unit='rad', rotation_sequence='ZYX'):
    # From navpy outputs x,y,z angles
    if(C.shape[0] != C.shape[1]):
        raise ValueError('Matrix is not square')
    if(C.shape[0] != 3):
        raise ValueError('Matrix is not 3x3')

    if(rotation_sequence == 'ZYX'):
        rotAngle1 = np.arctan2(C[0, 1], C[0, 0])   # Yaw
        rotAngle2 = -np.arcsin(C[0, 2])  # Pitch
        rotAngle3 = np.arctan2(C[1, 2], C[2, 2])  # Roll

    else:
        raise NotImplementedError('Rotation sequences other than ZYX are not currently implemented')

    if(output_unit == 'deg'):
        rotAngle1 = np.rad2deg(rotAngle1)
        rotAngle2 = np.rad2deg(rotAngle2)
        rotAngle3 = np.rad2deg(rotAngle3)

    return rotAngle3, rotAngle2, rotAngle1


