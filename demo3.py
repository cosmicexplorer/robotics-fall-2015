#!/usr/bin/env python

import rospy
import numpy
from numpy import matrix
from baxter_interface import CameraController
from sensor_msgs.msg import Image
import struct
import sys
import tf
import math
import time
import convert_notes
import numpy
from numpy import *
from numpy.linalg import *
from scipy import linalg
from tf import transformations
from sensor_msgs.msg import (
    JointState
)
from baxter_core_msgs.msg import(
    JointCommand,
)
import util
import time

def C(theta):
    return math.cos(theta)

def S(theta):
    return math.sin(theta)

def R_norm(R):
    aa = -numpy.cross((R[0,2],R[1,2],R[2,2]), (R[0,1],R[1,1],R[2,1]))
    bb = numpy.cross((R[0,2],R[1,2],R[2,2]), aa)
    aa = numpy.multiply(1/numpy.linalg.norm(aa), aa)
    bb = numpy.multiply(1/numpy.linalg.norm(bb), bb)
    cc = numpy.multiply(1/numpy.linalg.norm((R[0,2],R[1,2],R[2,2])), (R[0,2],R[1,2],R[2,2]))
    return numpy.column_stack((numpy.transpose(aa),numpy.transpose(bb), numpy.transpose(cc)))

def R2rpy(R):
    alpha = math.atan2(R[1,0], R[0,0]) # rot x (torso)
    beta = math.atan2(-1*R[2,0], math.sqrt(R[2,1]**2+R[2,2]**2)) # rot y
    gamma = math.atan2(R[2,1], R[2,2]) # rot z
    return [[alpha],[beta],[gamma]]

def R2axisang(R): # note only returns k in this case
    theta = math.acos((numpy.trace(R)-1)/2)
    if numpy.abs(theta-0.01) < 0:
        k = [[0],[0],[0]]
    else:
        k = numpy.multiply(1/(2*math.sin(theta)), [[R[2,1]-R[1,2]],[R[0,2]-R[2,0]],[R[1,0]-R[0,1]]])
    return k

def mag_speed(speed_max, speed_min, delta, eps, Lambda):
    if Lambda == 1:
        return 0
    if delta/eps > Lambda:
        return speed_max
    else:
        return (speed_max - speed_min) / (eps*(Lambda-1)) * (delta-eps) + speed_min

def A_calc(theta,a,alpha,d):
    return [[C(theta),-1*S(theta)*C(alpha),S(theta)*S(alpha),a*C(theta)],[S(theta),C(theta)*C(alpha),-1*C(theta)*S(alpha),a*S(theta)],[0,S(alpha),C(alpha),d],[0,0,0,1]]

def DH(THETA, A, ALPHA, D):
    T_RA = [[0.70711, 0.70711, 0, 0.064027],[-0.70711, 0.70711, 0, -0.25903],[0, 0, 1, 0.12963],[0, 0, 0, 1]]
    #T_LA = [[0.70711,-0.70711, 0, 0.064027],[ 0.70711, 0.70711, 0,  0.25903],[0, 0, 1, 0.12963],[0, 0, 0, 1]]
    T = numpy.zeros((4,4,7))
    T[:,:,0] = numpy.dot(T_RA, A_calc(THETA[0], A[0], ALPHA[0], D[0]))
    for i in xrange(1,len(THETA)):
        T[:,:,i] = numpy.dot(T[:,:,i-1], A_calc(THETA[i], A[i], ALPHA[i], D[i]))
    return T

def hat(H):
    if len(H) == 6:
        return numpy.vstack((numpy.column_stack((hat(H[3:7]), H[0:3])), [0,0,0,0]))
    else:
        return [[0,-H[2],H[1]],[H[2],0,-H[0]],[-H[1],H[0],0]]

def xi_rev(w,q):
    return numpy.vstack((numpy.transpose(matrix(numpy.cross(-w, q))), numpy.transpose(matrix(w))))

def baxter_fk(q):
    w = numpy.array([[0, 0.708733236,  0.705474709,    0.708729912,    0.70547736, 0.708723667,  0.705482832],
                     [0, 0.705476577, -0.70873136,     0.705478248,   -0.70873152, 0.705475767, -0.708726595],
                     [1, 0,            0.00230096915,  0.00153397613,  0.00115049, 0.003834937, -0.000766974859]])


    ax_pt = numpy.array([[0.06402724,   0.112705124,  0.184663544,  0.369981239,  0.44306164,  0.634069656, 0.715888027],
                         [-0.25902738, -0.307929978, -0.380220576, -0.566243712, -0.63966121, -0.831495933,-0.913690499],
                         [ 0.129626,    0.399976,     0.400210699,  0.331814783,  0.33193396,  0.322245479, 0.322156529]])

    gsj0 = numpy.array(zeros((4,4,len(q)+1)))
    gsj0[:,:,0] = numpy.array([[ 0.70547658, 0.70873324, 0, 0.06402724],
                               [-0.70873324, 0.70547658, 0,-0.25902738],
                               [ 0,          0,          1, 0.129626],
                               [ 0,          0,          0, 1]])


    gsj0[:,:,1] = numpy.array([[ 0.705474709,   0.00162327984,  0.708733236,  0.112705124],
                               [-0.70873136,   -0.00163077331,  0.705476577, -0.307929978],
                               [ 0.00230096915,-0.999997353,    0,            0.399976],
                               [ 0,             0,              0,            1]])


    gsj0[:,:,2] = numpy.array([[ 0.00271046068,   0.708729912,   0.705474709,    0.184663544],
                               [-0.000548584302,  0.705478248,  -0.70873136,    -0.380220576],
                               [-0.999996176,     0.00153397613, 0.00230096915,  0.400210699],
                               [ 0,               0,             0,              1]])


    gsj0[:,:,3] = numpy.array([[0.705477361,   0.00189882057,   0.708729912,   0.369981239],
                               [-0.708731522,  0.000266801104,  0.705478248,  -0.566243712],
                               [0.00115048669,-0.999998162,     0.00153397613, 0.331814783],
                               [0,             0,               0,             1]])


    gsj0[:,:,4] = numpy.array([[0.00352958,   0.70872367,  0.70547736,  0.44306164],
                               [0.00189008,   0.70547577, -0.70873152, -0.63966121],
                               [-0.99999198,  0.00383494,  0.00115049,  0.33193396],
                               [0,            0,           0,           1]])


    gsj0[:,:,5] = numpy.array([[0.705482832,      0.00217683966,  0.708723667,  0.634069656],
                               [-0.708726595,     0.00324905545,  0.705475767, -0.831495933],
                               [-0.000766974864, -0.999992352,    0.003834937, 0.322245479],
                               [0,                0,              0,           1]])


    gsj0[:,:,6] = numpy.array([[0.00108966904,  0.708726172,    0.705482832,     0.715888027],
                               [0.00216686578,  0.705479921,   -0.708726595,    -0.913690499],
                               [-0.999997059,   0.00230096403, -0.000766974859,  0.322156529],
                               [0,              0,              0,               1]])

    gsj0[:,:,7] = numpy.array([[0.00108966904,  0.708726172,    0.705482832,    0.795995603],
                               [0.00216686578,  0.705479921,   -0.708726595,   -0.994166404],
                               [-0.999997059,   0.00230096403, -0.000766974859, 0.322069439],
                               [0,              0,              0,              1]])
    sz = numpy.shape(w)
    xi = numpy.matrix(numpy.zeros((6,len(q))))
    H = numpy.zeros((4,4,len(q)+1))
    xi[:,0] = xi_rev(w[:,0],ax_pt[:,0])
    H[:,:,0] = linalg.expm(numpy.multiply(q[0],hat(xi[:,0])))
    for i in xrange(1, sz[1]):
        xi[:,i] = xi_rev(w[:,i],ax_pt[:,i])
        H[:,:,i] = dot(H[:,:,i-1],linalg.expm(numpy.multiply(q[i],hat(xi[:,i]))))

    H[:,:,7] = dot(H[:,:,6],gsj0[:,:,7])

    for i in xrange(0,sz[1]):
        H[:,:,i] = dot(H[:,:,i], gsj0[:,:,i])

    #H[:,:,6] = dot(H[:,:,6],gsj0[:,:,7])
    return H

def generate_joint_names_for_arm(arm):
    exts = ['s0','s1','e0','e1','w0','w1','w2']
    return map(lambda ext: arm + '_' + ext, exts)

def send_to_joint_vals(q, arm='right'):
    pub_joint_cmd=rospy.Publisher('/robot/limb/' + arm + '/joint_command',JointCommand)
    command_msg=JointCommand()
    command_msg.names = generate_joint_names_for_arm(arm)
    command_msg.command=q
    command_msg.mode=JointCommand.POSITION_MODE
    control_rate = rospy.Rate(100)
    start = rospy.get_time()

    joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
    qc = joint_positions.position[9:16]
    qc = ([qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6]])

    print('q,qc')
    print(q)
    print(qc)
    while not rospy.is_shutdown() and numpy.linalg.norm(numpy.subtract(q,qc))> 0.07:
        pub_joint_cmd.publish(command_msg)    # sends the commanded joint values to Baxter
        control_rate.sleep()
        joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
        qc = (joint_positions.position[9:16])
        qc = (qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6])
        print "joint error = ", numpy.linalg.norm(numpy.subtract(q,qc))
        print((q,qc))
    print("In home pose")
    return (qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6])

def getpose():
    pub_joint_cmd=rospy.Publisher('/robot/limb/right/joint_command',JointCommand)
    command_msg=JointCommand()
    command_msg.names=['right_s0', 'right_s1', 'right_e0', 'right_e1',  'right_w0', 'right_w1', 'right_w2']
    command_msg.mode=JointCommand.POSITION_MODE
    control_rate = rospy.Rate(100)
    start = rospy.get_time()

    joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
    qc = (joint_positions.position[9:16])
    angles = [qc[2],qc[3],qc[0],qc[1],qc[4],qc[5],qc[6]]
    pub_joint_cmd=rospy.Publisher('/robot/limb/right/joint_command',JointCommand)
    command_msg=JointCommand()
    command_msg.names=['right_e1']
    command_msg.command=[0]
    command_msg.mode=JointCommand.POSITION_MODE
    control_rate = rospy.Rate(100)
    start = rospy.get_time()
    exit = 0
    listener2=tf.TransformListener()

    #the transformations
    now=rospy.Time()
    listener2.waitForTransform("/torso","/right_hand",now,rospy.Duration(1.0))
    (trans08,rot08)=listener2.lookupTransform("/torso","/right_hand",now)

    # Get 4*4 rotational matrix from quaternion ( 4th colume is [0][0][0][1])
    R08 = transformations.quaternion_matrix(rot08)
    T08 = numpy.vstack((numpy.column_stack((R08[0:3,0:3], numpy.transpose(numpy.array(trans08)))),[0,0,0,1]))
    return (angles, T08)

def Jv(rho,z,on,o):
    #print "inside Jv =",numpy.add(numpy.transpose([numpy.multiply(rho,numpy.cross(z,numpy.subtract(on,o)))]),numpy.transpose([numpy.multiply(rho-1,z)]))
    return numpy.add(numpy.transpose([numpy.multiply(rho,numpy.cross(z,numpy.subtract(on,o)))]),numpy.transpose([numpy.multiply(rho-1,z)])) # Jv = z_(i-1)X(O_n - O_(i-1)) for revolute joint, i, and Jv = z_(i-1) for prismatic joint, i.

def Jw(rho,z):
    return numpy.transpose([numpy.multiply(rho,z)]) # Jw = z for revolute joint, i, and Jw = [0;0;0] for prismatic joint, i.

def Jg(Jv,Jh):
    return numpy.vstack((Jv,Jh)) # Put the two halves of the Gemetric Jacobian togeter, could be called Jh = Jg(Jv(rho,z,on,o),Jw(rho,z));

def calc_Jg(frames):
    sz = numpy.shape(frames)
    O = numpy.dot(frames[:,:,0],[[0],[0],[0],[1]])
    z = numpy.dot(frames[:,:,0],[[0],[0],[1],[0]])
    for i in range(1, sz[2]):
        O = numpy.column_stack((O, numpy.dot(frames[:,:,i],[[0],[0],[0],[1]])))
        z = numpy.column_stack((z, numpy.dot(frames[:,:,i],[[0],[0],[1],[0]])))
    z = z[0:3,:]
    O = O[0:3,:]
    rho = numpy.ones(sz[2]) # rho_1...6 values
    J_v = Jv(rho[0], z[:,0],O[:,sz[2]-1],O[:,0])
    J_w = Jw(rho[0], z[:,0])

    for i in range(1,sz[2]-1): # calculate the Jacobian column by column
        J_v = numpy.column_stack((J_v, Jv(rho[i], z[:,i],O[:,sz[2]-1],O[:,i])))
        J_w = numpy.column_stack((J_w, Jw(rho[i], z[:,i])))
    J_g = Jg(J_v,J_w)
    return J_g

def inv_kin(Rot,XYZ,q):
    Rot[0:3,0:3] = R_norm(Rot[0:3,0:3])

    # Pull out rotation matrix for target pose
    Rd = Rot
    Pd = XYZ

    lambda_v = 100
    v_max = 0.05
    v_min = 0.0024
    delta_p = 1
    eps_p = 0.005
    eta_p = lambda_v*eps_p

    ksi_d = R2rpy(Rd)
    eps_ksi = 0.1
    lambda_ksi = 10
    ksi_d_max = 0.5
    ksi_d_min = 0.01
    delta_ksi = 1
    eta_ksi = lambda_ksi*eps_ksi
    start=rospy.get_time()
    while (delta_p>eps_p):
        T = baxter_fk(q)
        # current rotation
        Rc = T[0:3,0:3,7]

        # current position
        Pc = T[0:3,3,7]

        # current Jacobian and pseudoinversed Jacobian
        Jg = calc_Jg(T)
        Jgp = numpy.linalg.pinv(Jg)

        # desired position calc
        Pd_Pc_diff = numpy.subtract(Pd,Pc)
        delta_p = numpy.linalg.norm(Pd_Pc_diff)

        n_hat = numpy.multiply(1/delta_p,Pd_Pc_diff)
        Pd_dot = numpy.transpose([numpy.multiply(mag_speed(v_max, v_min, delta_p, eps_p, lambda_v), n_hat)])

        # current angle calc
        ksi_c = R2rpy(Rc) # current orientation (rpy)
        Re = R_norm(dot(Rd,numpy.transpose(Rc))) # error rotation matrix
        mhat_e = R2axisang(Re)
        delta_ksi  = numpy.linalg.norm(numpy.subtract(ksi_d, ksi_c))
        ksi_d_dot = numpy.multiply(mag_speed(ksi_d_max, ksi_d_min, delta_ksi, eps_ksi, lambda_ksi), mhat_e)
        x_d_dot = numpy.vstack((Pd_dot,ksi_d_dot))
        q_dot = list(numpy.array(numpy.transpose(dot(Jgp,x_d_dot))).reshape(-1))
        # change the data type so that the command message will accept the new q
        # value
        q = numpy.add(q, q_dot)
    return q

angular_tol = .1
qMax = [.89, 1.047, 3.028, 2.618, 3.059, 2.094, 3.059]
qMin = [-2.461, -2.147, -3.028, -.052, -3.059, -1.571, -3.059]

def get_joint_values(arm):
    posVec = rospy.wait_for_message("/robot/joint_states", JointState).position
    qc = []
    if 'left' == arm:
        qc = posVec[2:8]
    elif 'right' == arm:
        qc = posVec[9:16]
    else:
        raise RuntimeError(arm + " is invalid arm!")
    qc = ([qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6]])
    return qc

def get_trans(arm):
    listener = tf.TransformListener()
    now = rospy.Time()
    listener.waitForTransform("/torso", "/" + arm + "_hand",
                              now, rospy.Duration(1.0))
    trans, rot = listener.lookupTransform("/torso", "/" + arm + "_hand", now)
    rot_4x4 = transformations.quaternion_matrix(rot)
    t = numpy.vstack((numpy.column_stack((rot_4x4[0:3,0:3],
                                          numpy.transpose(numpy.array(trans)))),
                      [0,0,0,1]))
    return t

def rot_pos(trans):
    return (trans[0:3,0:3], trans[0:3,3])

def mid_to_up_or_down(down, trans):
    new_trans = down['trans'] * trans
    print('mid[\'trans\']')
    print(down['trans'])
    print('trans')
    print(trans)
    print('new_trans')
    print(new_trans)
    rot, pos = rot_pos(new_trans)
    new_pos = inv_kin(numpy.array(rot), numpy.array(pos.T)[0], down['q'])
    return {
        'q': tuple(new_pos),
        'trans': numpy.matrix(new_trans)
    }

# init_pos is the position of baxter at the first key, pressing down, as joints
# key_delta is a transformation matrix, as is up_z
def generate_q_for_keys(cur_pos, cur_trans, num_keys, key_delta, up_z, down_z, arm):
    mids = []
    cur_trans = numpy.matrix(cur_trans)
    key_delta = numpy.matrix(key_delta)
    for i in range(0, num_keys):
        mids.append({
            'q': tuple(cur_pos),
            'trans': numpy.matrix(cur_trans)
        })
        print(cur_trans)
        print(cur_pos)
        cur_trans = cur_trans * key_delta
        rot, pos = rot_pos(numpy.array(cur_trans))
        cur_pos = inv_kin(rot, pos, cur_pos)
    ups = map(lambda el: mid_to_up_or_down(el, up_z), mids)
    downs = map(lambda el: mid_to_up_or_down(el, down_z), mids)
    return (mids, downs, ups)

key_delta_pos = [0, .0762, 0]
# key_delta_pos = [0, .03, 0]
key_delta = util.transformation(numpy.eye(3), key_delta_pos)
up_z_delta_pos = [0, 0, -.05]
up_z_delta = util.transformation(numpy.eye(3), up_z_delta_pos)
down_z_delta_pos = [0, 0, .015]
down_z_delta = util.transformation(numpy.eye(3), down_z_delta_pos)

NUM_KEYS = 8

def move_to_key(from_key, to_key, key_up_qs, key_down_qs):
    send_to_joint_vals(key_up_qs[from_key])
    send_to_joint_vals(key_up_qs[to_key])
    time.sleep(convert_notes.sleep_time)
    send_to_joint_vals(key_down_qs[to_key])

def step_off_key(from_key, key_up_qs):
    send_to_joint_vals(key_up_qs[from_key])

def run_song(songfile, cur_key, key_up_qs, key_down_qs):
    movements_array = convert_notes.parse_file(songfile)
    control_rate = rospy.Rate(1./convert_notes.smallest_quant)
    for action in movements_array:
        if action['type'] == 'move-and-press':
            move_to_key(cur_key, action['num'] - 1, key_up_qs, key_down_qs)
            cur_key = action['num'] - 1
        elif action['type'] == 'off':
            step_off_key(cur_key, key_up_qs)
        control_rate.sleep()

initial_key = 0

def main():
    rospy.init_node("baxter_kinematics")
    raw_input("press enter to read joint values and transformation")
    init_q = get_joint_values('right')
    init_trans = get_trans('right')
    print(init_q)
    print(init_trans)
    key_mids, key_downs, key_ups = generate_q_for_keys(
        init_q, init_trans, NUM_KEYS, key_delta, up_z_delta,
        down_z_delta, 'right')
    songfile = raw_input("specify song file: ")
    if songfile == '':
        songfile = 'jingle.txt'
    with open('out_q','w') as f:
        f.write(repr(key_mids[0]['q']) + '\n')
    send_to_joint_vals(key_ups[initial_key]['q'])
    run_song('pianist/' + songfile, initial_key,
             map(lambda up: up['q'], key_ups),
             map(lambda down: down['q'], key_downs))
    print(key_mids[0]['q'])
    print('bye!')

if __name__ == '__main__':
    main()
