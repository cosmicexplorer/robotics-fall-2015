import numpy
import numpy.linalg

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
# J is the hubrid jocabian matrix (6 x n)
# T is a transformation from the base frame representing current pos/ori (6 x n)
# aka transform from base to end effector
# target is a transformation from the base frame (pos/ori) (6 x n)
def resolvedRates(J, T, target, speed):
    linDistance = numpy.matrix(target[0:3,0]-T[0:3,3])
    angDistance = numpy.matrix(target[3:,0]-dcm2angle(T[0:3,0:3]))
    linVelocity = speed[0]*(linDistance/numpy.linalg.norm(linDistance))
    angVelocity = speed[1]*(angDistance/numpy.linalg.norm(angDistance))
    velocity = numpy.r_[linVelocity,angVelocity] # 6 x 1 in the hubrid frem!!
    # velocity = numpy.r_[linVelocity, numpy.matrix([[0],[0],[0]])]
    qdot = numpy.linalg.pinv(J)*velocity
    return qdot

def quat2rot(qx,qy,qz,qw):
    return numpy.matrix(
        [[1-2*(qy**2)-2*(qz**2), 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*(qx**2)-2*(qz**2), 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*(qx**2)-2*(qy**2)]])

def transformation(R,p):
    T = numpy.c_[R, p]
    return numpy.r_[T,[[0, 0, 0, 1]]]

def dcm2angle(C, output_unit='rad', rotation_sequence='ZYX'):
    # From navpy outputs x,y,z angles
    if(C.shape[0] != C.shape[1]):
        raise ValueError('Matrix is not square')
    if(C.shape[0] != 3):
        raise ValueError('Matrix is not 3x3')

    if(rotation_sequence == 'ZYX'):
        rotAngle1 = numpy.arctan2(C[0, 1], C[0, 0])   # Yaw
        rotAngle2 = -numpy.arcsin(C[0, 2])  # Pitch
        rotAngle3 = numpy.arctan2(C[1, 2], C[2, 2])  # Roll
    else:
        raise NotImplementedError('Rotation sequences other than ' +
		' ZYX are not currently implemented')

    if(output_unit == 'deg'):
        rotAngle1 = numpy.rad2deg(rotAngle1)
        rotAngle2 = numpy.rad2deg(rotAngle2)
        rotAngle3 = numpy.rad2deg(rotAngle3)

    return numpy.matrix([[rotAngle3], [rotAngle2], [rotAngle1]])

def zeroOutColOfMatrix(mat, col):
    retmat = numpy.copy(mat)
    for row in range(0, mat.shape[0]): retmat[row, col] = 0
    return retmat

def resolvedRatesWithLimits(J, T, target, speed, cur_q, freq, qMax, qMin, tol):
    numLinks = len(qMax)
    numFailures = 0
    my_j = numpy.copy(J)

    while numFailures < numLinks - 3:
        q_dot = resolvedRates(my_j, T, target, speed)
        out = list(numpy.array(cur_q + (1./freq) * q_dot.T)[0])
        failures = []
        for i in range(0, numLinks):
            if ((out[i] < qMin[i] + tol) or 
                (out[i] > qMax[i] - tol)):
                failures.append(i)
        if len(failures) == 0: return (q_dot, out)
        curFail = failures[0]
        my_j = zeroOutColOfMatrix(my_j, curFail)
        numFailures = numFailures + 1
    # if everything locked up, go to middle for all joints
    middles = map(lambda (a, b): a + b / 2, zip(qMax, qMin))
    return (None, middles)

# A Realistic Joint Limit Algorithm for Kinematically Redundant Manipulators
def jointLimitPaper(theta, thetamin, thetamax,alpha,J,k):
    # theta is an array of current joint angles (nx1)
    # thetamin/max are arrays of joint limits for each joint (nx1)
    # alpha is the tolerance angle
    # J is the current jacobian
    # k is a scaling factor
    dp_dtheta = []
    for i in range(0, len(theta)) :
        ei = thetamax[i]- thetamin[i]
        dp1_dtheta = 0
        if theta[i]<thetamin[i]+alpha:
            dp1_dtheta = 2*(theta[i]/(ei**2))
        dp2_dtheta = 0
        if theta[i]>thetamax[i]-alpha:
            dp2_dtheta = 2*(theta[i]/(ei**2))
        dp_dtheta.append(dp1_dtheta+dp2_dtheta)
    pinv_times_J = numpy.linalg.pinv(J)*J
    dp_dtheta_vect = numpy.matrix([dp_dtheta]).T
    return (numpy.identity(pinv_times_J.shape[0])-pinv_times_J)*(-k*dp_dtheta_vect)
