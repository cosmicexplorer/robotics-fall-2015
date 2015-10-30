# simulate the effects of resolved rates

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

startRotTest = numpy.matrix([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
startPointTest = numpy.matrix([1, 2, 3]).T
startTest = makeTransformMat(startRotTest, startPointTest)

endRotTest = numpy.matrix([[1, 0, 0],
                       [0, numpy.sqrt(3) / 2, 1/2],
                       [0, -1/2, numpy.sqrt(3)/2]])
endPointTest = numpy.matrix([-3, -2, -1]).T
endTest = makeTransformMat(endRotTest, endPointTest)

def makeTrajectory(start, end, timesteps):
    for t in range(0, timesteps):
        yield start + (end - start) * (t / timesteps)
