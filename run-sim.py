# run simulation in simulate.py

import numpy
import util
import simulate

startRot = numpy.matrix([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
startPoint = numpy.matrix([1, 2, 3]).T
start = util.makeTransformMat(startRot, startPoint)

endRot = numpy.matrix([[1, 0, 0],
                       [0, numpy.sqrt(3) / 2, 1/2],
                       [0, -1/2, numpy.sqrt(3)/2]])
endPoint = numpy.matrix([-3, -2, -1]).T
end = util.makeTransformMat(endRot, endPoint)

timesteps = 1000

def makeTrajectoryTest(start, end, timesteps):
    for t in range(0, timesteps):
        yield start + (end - start) * ((t + 1) / timesteps)

trajectory = makeTrajectoryTest(start, end, timesteps)
