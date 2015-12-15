import numpy
import numpy.matlib

#def generateQ(thetaCur,posFinal, freq)
def main():
    Rdes = RotY(numpy.pi)

    dt = 1/freq
    w = numpy.matrix([[0,0,1,0,1,0,1],[0,1,0,1,0,1,0],[1,0,0,0,0,0,0]])
    q = numpy.matrix([[0,0.069,0.171,0.4334,0.537,0.8077,0.9237],
                      [0,0,0,0,0,0,0],
                      [0,0.2703,0.2703,0.2014,0.2014,0.1913,0.1913]])

    tw = numpy.matlib.zeros((6,7))
    pos = numpy.matlib.zeros((3,7))
    for i in range(0,6):
        tw[:,i:(i+1)] = numpy.vstack((-numpy.cross(w[:,i],q[:,i],axis=0),w[:,i]))
        pos[:,i:(i+1)]= [[q[0,i]],[q[1,i]],[q[2,i]]]
    print pos
        
    g01 = numpy.identity(4)
    g02 = transformation(Rotx(-(numpy.pi)/2),pos[:,1:2])
    g03 = transformation()
    
def transformation(R,p):
    T = numpy.c_[R, p]
    return numpy.r_[T,[[0, 0, 0, 1]]]
