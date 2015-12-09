import numpy
import numpy.matlib

#def generateQ(thetaCur,posFinal, freq)
def main():
    freq = 100

    Rdes = rotY(numpy.pi)

    dt = 1/freq
    w = numpy.matrix([[0,0,1,0,1,0,1],[0,1,0,1,0,1,0],[1,0,0,0,0,0,0]])
    qtw = numpy.matrix([[0,0.069,0.171,0.4334,0.537,0.8077,0.9237],
                      [0,0,0,0,0,0,0],
                      [0,0.2703,0.2703,0.2014,0.2014,0.1913,0.1913]])

    tw = numpy.matlib.zeros((6,7))
    pos = numpy.matlib.zeros((3,7))
    for i in range(0,6):
        tw[:,i:(i+1)] = numpy.vstack((-numpy.cross(w[:,i],qtw[:,i],axis=0),w[:,i]))
        pos[:,i:(i+1)]= [[qtw[0,i]],[qtw[1,i]],[qtw[2,i]]]
        
    g01 = numpy.identity(4)
    g02 = transformation(rotX(-(numpy.pi)/2),pos[:,1:2])
    g03 = transformation(rotY((numpy.pi)/2),pos[:,2:3])
    g04 = transformation(rotX(-(numpy.pi)/2),pos[:,3:4])
    g05 = transformation(rotY((numpy.pi)/2),pos[:,4:5])
    g06 = transformation(rotX(-(numpy.pi)/2),pos[:,5:6])
    g07 = transformation(rotY((numpy.pi)/2),pos[:,6:7])
    g0e = transformation(rotY((numpy.pi)/2),numpy.matrix([[1.0374],[-0.022],[0.1924]]))

    while running:
        ex = numpy.matlib.zeros((1,7))
        H = numpy.matlib.zeros((1,7))
        for i in range(0,6):
            ex[i,:]= numpy.scipy.expm(util.hat6(tw[:,i])*q(i))

        
        # calculate jacobian J, H stuff left out.. needs to be put in
        Oe = He[0:3,3]
        O = numpy.matlib.zeros((3,7))
        
        CurrR = He[0:3,0:3]
        ErrR  = numpy.dot(invt(CurrR),Rdes)
        theta=acos((numpy.trace(Re)-1)/2)  
        me=(1/(2/sin(theta)))*asarray([[Re[2,1]-Re[1,2]],[Re[0,2]-Re[2,0]],[Re[1,0]-Re[0,1]]]) ##error in ori

        w_unit = me
        EdotR = 0.001*w_unit
        
        
        

    while
def transformation(R,p):
    T = numpy.c_[R, p]
    return numpy.r_[T,[[0, 0, 0, 1]]]

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
main()

