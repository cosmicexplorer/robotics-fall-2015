# run simulation in simulate.py

import numpy
import util

startRotTest = numpy.matrix([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
startPointTest = numpy.matrix([1, 2, 3]).T
startTest = util.makeTransformMat(startRotTest, startPointTest)

endRotTest = numpy.matrix([[1, 0, 0],
                       [0, numpy.sqrt(3) / 2, 1/2],
                       [0, -1/2, numpy.sqrt(3)/2]])
endPointTest = numpy.matrix([-3, -2, -1]).T
endTest = util.makeTransformMat(endRotTest, endPointTest)
