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
def resolvedRates(J, T, target):
    # need to calc trajectory; straight line? how to avoid singularities?
    # how to later add in collision detection?
