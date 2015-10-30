import numpy

import util

def logtr(g):
    R = g[0:3,0:3]
    theta = numpy.arccos((numpy.trace(R)-1)/2)
    p = g[0:3,3]

    if numpy.absolute(theta) < 1e-8:
        w = numpy.transpose(numpy.matrix([0, 0, 0]))
        v = p
    else:
        w = numpy.zeros((3,1))
        w(0) = 1/2/numpy.sin(theta)*(R[2,1]-R[1,2])
        w(1) = 1/2/numpy.sin(theta)*(R[0,2]-R[2,0])
        w(2) = 1/2/numpy.sin(theta)*(R[1,0]-R[0,1])

        A = (numpy.identity(3)-R)*util.hat3(w)+w*w.T*theta
        v = numpy.linalg.solve(A,p)
        w = w*theta
        v = v*theta

    return util.hat6(numpy.vstack(v,w))
