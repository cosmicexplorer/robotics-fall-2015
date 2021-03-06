import numpy
import numpy.matlib

def hat3(v):
    return numpy.matrix([[0,-v[2],v[1]],
                         [v[2],0,-v[0]],
                         [-v[1],v[0],0]])
def hat6(v):
    return numpy.matrix([[0,-v[5],v[4],v[0]],
                         [v[5],0,-v[3],v[1]],
                         [-v[4],v[3],0,v[2]],
                         [0,0,0,0]])
def wedge3(vhat):
    return numpy.array([vhat[2,1],vhat[0,2],vhat[1,0]])

def wedge6(vhat):
    return numpy.array(
        [vhat[0,3], vhat[1,3], vhat[2,3],
         vhat[2,1], vhat[0,2], vhat[1,0]])

# 6x1 [x,y,z,theta_x,theta_y,theta_z]
def posOriToTransformMat(V):
    R = angle2dcm(V[3:,0])
    p = V[:3,0]
    return transformation(R, p)

# alpha is scalar
# tol is scalar
# getJacobian is function which accepts no arguments
# waiter is an object with a sleep function which accepts no arguments
# angles_get fun, no args
# getT: fun, no args
# posori_get: fun, takes transformation matrix, returns 6x1 pos/ori vector
# publish_fun: recieve desired q
# x_des: target position
# dt: delta time        err = [final(1)-Oe(1);final(2)-Oe(2);final(3)-Oe(3)];
def resolvedRatesorig(alpha, tol, getJacobian, waiter, angles_get, getT, posori_get, publish_fun, x_des, speed, dt):
    # linear trajectory, so no explicit trajectory array
    # drawn directly from solutions for assignment 5
    t = 0
    T = getT()
    x_cur = posori_get(T)
    x0 = x_cur
    x_err = x_des - x_cur
    v_des = speed * x_err / numpy.linalg.norm(x_err)

    # v_matrix_des = speed *
    # waiter should be NEWLY CONSTRUCTED
    waiter.sleep()              # start off sleeping

    while numpy.linalg.norm(x_err) > tol:
	if numpy.linalg.norm(x_err[0:3]) < tol:
		v_des[0:3,0]=0
	g_inv = invT(T)
        g_d = posOriToTransformMat(x0 + v_des * t)
        p_cur_err = transformMatToPosOri(logtr(g_inv * g_d))
        J = getJacobian()
        # FIXME: v_des is 6x1, p_cur_err is 4x4 (from logtr, which calls
        # hat6). this is a problem.
        q_dot = numpy.linalg.pinv(J) * (v_des + alpha * p_cur_err)
        # TODO: call correction function here to modify q_dot, if applicable
        q = angles_get() + q_dot * dt
        t += dt
        publish_fun(q)
	T
        waiter.sleep()
        T = getT()
        x_cur = posori_get(T)
        x_err = x_des - x_cur

def resolvedRates(alpha, tol, getJacobian, waiter, angles_get, getT, posori_get, publish_fun, x_des, speed, dt):

    waiter.sleep()              # start off sleeping
    test = True
    q = angles_get()
    while test:
        T = getT()
	#print T

    	pos_cur = T[0:3,3]
    	pos_des = x_des[0:3,0]
	#print pos_cur
	#print pos_des
    	pos_err = pos_des-pos_cur
	#print pos_err



        npos_err = (numpy.linalg.norm(pos_err))
	npos_err
    	# need to tune these constants
    	v = 0.1*pos_err


    	R_cur = T[0:3,0:3]
    	# hardcoded in now for flat orientation
    	R_des = rotY(numpy.pi/2)
    	R_err=numpy.dot(R_des,numpy.transpose(R_cur))
    	k=numpy.arccos((numpy.trace(R_err)-1)/2)
    	rot_err=(1./(2./numpy.sin(k)))*numpy.matrix([[R_err[2,1]-R_err[1,2]],[R_err[0,2]-R_err[2,0]],[R_err[1,0]-R_err[0,1]]]) ##error in orientation
	nrot_err = (numpy.linalg.norm(rot_err))
	# need to tune these constants
    	w = 0.1*k*rot_err
	w = numpy.matlib.zeros((3,1))
	nrot_err = tol

	if (npos_err>tol) and (nrot_err>tol):
		donothing = 0
		print('hi')
	elif (npos_err>tol) and (nrot_err<tol):
		w = numpy.matlib.zeros((3,1))
	elif (npos_err<tol) and (nrot_err>tol):
		v = numpy.matlib.zeros((3,1))
	elif (npos_err<tol) and (nrot_err<tol):
		w = numpy.matlib.zeros((3,1))
		v = numpy.matlib.zeros((3,1))
		test = False

	J = getJacobian()
	#print J
	#pinv = numpy.transpose(J)*(J*numpy.transpose(J)+0.01*numpy.eye(6,6))**(-1)
	pinv =numpy.linalg.pinv(J)
	#print numpy.vstack((v,w))
        q_dot = pinv*(numpy.vstack((v,w)))
	qprev = q
	q= qprev + q_dot*dt
	print q
	#q = ([q[2], q[3], q[0], q[1], q[4], q[5], q[6]]) # reorder due to the odd order that q is read in from the arm

        publish_fun(q)
        waiter.sleep()


def quat2rot(qx,qy,qz,qw):
    return numpy.matrix(
        [[1-2*(qy**2)-2*(qz**2), 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*(qx**2)-2*(qz**2), 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*(qx**2)-2*(qy**2)]])

def transformation(R,p):
    T = numpy.c_[R, p]
    return numpy.r_[T,[[0, 0, 0, 1]]]

def angle2dcm(inVect, input_unit='rad',
              rotation_sequence='ZYX', output_type='matrix'):
    inVect *= -1
    rotAngle1 = inVect[0,0]
    rotAngle2 = inVect[1,0]
    rotAngle3 = inVect[2,0]
    # switch ZYX->XYZ
    tmp = rotAngle1
    rotAngle1 = rotAngle3
    rotAngle3 = tmp

    R3 = numpy.zeros((3, 3))
    R2 = numpy.zeros((3, 3))
    R1 = numpy.zeros((3, 3))
    if(input_unit == 'deg'):
        rotAngle1 = numpy.deg2rad(rotAngle1)
        rotAngle2 = numpy.deg2rad(rotAngle2)
        rotAngle3 = numpy.deg2rad(rotAngle3)
    R3[2, 2] = 1.0
    R3[0, 0] = numpy.cos(rotAngle1)
    R3[0, 1] = numpy.sin(rotAngle1)
    R3[1, 0] = -numpy.sin(rotAngle1)
    R3[1, 1] = numpy.cos(rotAngle1)
    R2[1, 1] = 1.0
    R2[0, 0] = numpy.cos(rotAngle2)
    R2[0, 2] = -numpy.sin(rotAngle2)
    R2[2, 0] = numpy.sin(rotAngle2)
    R2[2, 2] = numpy.cos(rotAngle2)
    R1[0, 0] = 1.0
    R1[1, 1] = numpy.cos(rotAngle3)
    R1[1, 2] = numpy.sin(rotAngle3)
    R1[2, 1] = -numpy.sin(rotAngle3)
    R1[2, 2] = numpy.cos(rotAngle3)
    if rotation_sequence == 'ZYX':
        C = R1.dot(R2.dot(R3))
    else:
        raise NotImplementedError('Rotation sequences other than ZYX are not currently implemented')
    if(output_type == 'matrix'): C = numpy.matrix(C)
    return C

def dcm2angle(C, output_unit='rad', rotation_sequence='ZYX'):
    # From navpy outputs x,y,z angles
    if(C.shape[0] != C.shape[1]):
        raise ValueError('Matrix is not square')
    if(C.shape[0] != 3):
        raise ValueError('Matrix is not 3x3')

    rotAngle1 = numpy.arctan2(C[0, 1], C[0, 0])   # Yaw
    rotAngle2 = -numpy.arcsin(C[0, 2])  # Pitch
    rotAngle3 = numpy.arctan2(C[1, 2], C[2, 2])  # Roll

    if(output_unit == 'deg'):
        rotAngle1 = numpy.rad2deg(rotAngle1)
        rotAngle2 = numpy.rad2deg(rotAngle2)
        rotAngle3 = numpy.rad2deg(rotAngle3)

    return -1 * numpy.matrix([[rotAngle3], [rotAngle2], [rotAngle1]])

def R2rpy(R):
	alpha = numpy.arctan2(R[1, 0], R[0, 0]) # rot x
	beta = numpy.arctan2(-1*R[2, 0], numpy.sqrt(R[2, 1]**2+R[2, 2]**2)) # rot y
	gamma = numpy.arctan2(R[2, 1], R[2, 2]) # rot z
	return numpy.matrix([[alpha],[beta],[gamma]])

def transformMatToPosOri(T):
    pos = T[:3,3]
    ori = R2rpy(T[:3,:3])
    return numpy.vstack([pos, ori])

def zeroOutColOfMatrix(mat, col):
    retmat = numpy.copy(mat)
    for row in range(0, mat.shape[0]): retmat[row, col] = 0
    return retmat

def resolvedRatesWithLimits(J, T, target, speed, cur_q, freq, qMax, qMin, tol):
    numLinks = len(qMax)
    numFails = 0
    my_j = numpy.copy(J)
    middles = map(lambda (a, b): (a + b) / 2, zip(qMax, qMin))

    while numFails < numLinks - 3:
        q_dot = resolvedRates(my_j, T, target, speed)
        out = list(numpy.array(cur_q + (1./freq) * q_dot.T)[0])
        failures = filter(lambda i: (out[i] < qMin[i] + tol) or
                                    (out[i] > qMax[i] - tol),
                          range(0, numLinks))
        if len(failures) == 0: return (q_dot, out)
        for fail in failures:
            numFails = numFails + 1
            my_j = zeroOutColOfMatrix(my_j, fail)
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
        ei = thetamax[i] - thetamin[i]
        dp1_dtheta = 0
        if theta[i]<thetamin[i]+alpha:
            c = thetamin[i] + alpha
            dp1_dtheta = 2*(theta[i]-c)/(ei**2)
        dp2_dtheta = 0
        if theta[i]>thetamax[i]-alpha:
            c = thetamax[i] - alpha
            dp2_dtheta = 2*(theta[i]-c)/(ei**2)
        dp_dtheta.append(dp1_dtheta+dp2_dtheta)
    pinv_times_J = numpy.matrix(numpy.linalg.pinv(J))*numpy.matrix(J)
    dp_dtheta_vect = numpy.matrix([dp_dtheta]).T
    return (numpy.identity(pinv_times_J.shape[0])-pinv_times_J)*(-k*dp_dtheta_vect)

def logtr(g):
    R = g[0:3,0:3]

    theta = numpy.arccos((numpy.trace(R)-1.)/2.)
    testval = numpy.absolute((numpy.trace(R)-1.)/2-1)
    if testval < .0000000001:
        theta=0
    p = g[0:3,3]

    if numpy.absolute(theta) < 1e-8:
        w = numpy.transpose(numpy.matrix([0, 0, 0]))
        v = p
    else:
        w = numpy.zeros((3,1))
        w[0] = 1./2./numpy.sin(theta)*(R[2,1]-R[1,2])
        w[1] = 1./2./numpy.sin(theta)*(R[0,2]-R[2,0])
        w[2] = 1./2./numpy.sin(theta)*(R[1,0]-R[0,1])

        A = (numpy.identity(3)-R)*hat3(w)+w*w.T*theta
        if numpy.linalg.det(A)==0:
            v=numpy.linalg.lstsq(A,p)
        else:
            v = numpy.linalg.solve(A,p)
        w = w*theta
        v = v*theta
    return numpy.vstack([v,w])

def invT(T):
    R = T[0:3,0:3]
    p = T[0:3,3]
    return transformation(R,p)

def rotX(theta):
    R = numpy.matrix([[1,0,0],
                      [0,numpy.cos(theta),-numpy.sin(theta)],
                      [0,numpy.sin(theta),numpy.cos(theta)]])
    return R

def rotY(theta):
    R = numpy.matrix([[numpy.cos(theta),0,numpy.sin(theta)],
                      [0,1,0],
                      [-numpy.sin(theta),0,numpy.cos(theta)]])
    return R
def rotZ(theta):
    R = numpy.matrix([[numpy.cos(theta),-numpy.sin(theta),0],
                     [numpy.sin(theta),numpy.cos(theta),0],
                     [0,0,1]])
