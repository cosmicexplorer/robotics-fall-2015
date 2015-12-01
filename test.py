#!/usr/bin/env python
import struct
import sys
import rospy
import tf
import numpy
import util
from baxter_pykdl import baxter_kinematics

from tf import transformations

from std_msgs.msg import (
    Float64,
)

from sensor_msgs.msg import (
    JointState
)

from baxter_core_msgs.msg import(
    JointCommand,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
def send_to_joint_vals(q):
    # Send Baxter to the home configuration
    # create joint commands for home pose
    pub_joint_cmd=rospy.Publisher(
        '/robot/limb/right/joint_command',JointCommand)   # setup the publisher
    command_msg=JointCommand()
    command_msg.names=['right_s0', 'right_s1', 'right_e0', 'right_e1',
                       'right_w0', 'right_w1', 'right_w2']
    command_msg.command=q
    command_msg.mode=JointCommand.POSITION_MODE
    control_rate = rospy.Rate(100) # sending commands at 100HZ

    # acquire the current joint positions
    joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
    qc = joint_positions.position[9:16]
    qc = ([qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6]]) # reorder due to the odd order that q is read in from the arm

    print(command_msg)
    while not rospy.is_shutdown() and numpy.linalg.norm(numpy.subtract(q,qc))>0.01: # move until the desired joint variable
        pub_joint_cmd.publish(command_msg)    # sends the commanded joint values to Baxter
        control_rate.sleep()                    # sending commands at 100HZ
        joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
        qc = (joint_positions.position[9:16])
        qc = (qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6])
        #print "joint error = ", numpy.linalg.norm(numpy.subtract(q,qc))
    print("In home pose")
    return (qc[2], qc[3], qc[0], qc[1], qc[4], qc[5], qc[6]) # reorder due to the odd order that q is read in from the arm

def getD(cur, final, scale):
    # FIXME: currently removing all angular distance stuff
    lin = numpy.subtract(final[0:3,0], cur[0:3,0])
    # ang = numpy.subtract(final[3:,0], cur[3:,0])
    lin_d = numpy.linalg.norm(lin)
    # ang_d = numpy.linalg.norm(ang)
    ang_d = 0
    return numpy.sqrt((lin_d * scale[0]) ** 2 + (ang_d * scale[1]) ** 2)

def runResolvedRates(rkin, final, speed, freq, scale, tolerance, qMax, qMin,
                     angular_tol):
    # stupid setup stuff
    pub_joint_cmd=rospy.Publisher(
        '/robot/limb/right/joint_command',JointCommand)
    command_msg = JointCommand()
    command_msg.names = ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                       'right_w0', 'right_w1', 'right_w2']
    command_msg.mode=JointCommand.POSITION_MODE
    control_rate = rospy.Rate(freq) # sending commands at freq HZ

    joint_states = rospy.wait_for_message("/robot/joint_states", JointState)
    cur_q = joint_states.position[9:16]
    cur_q = [cur_q[2], cur_q[3], cur_q[0], cur_q[1],
             cur_q[4], cur_q[5], cur_q[6]]
    forward = rkin.forward_position_kinematics()
    quatangles = forward[3:]
    rot = util.quat2rot(quatangles[0], quatangles[1], quatangles[2],
                        quatangles[3])
    pos = numpy.matrix(forward[:3]).T
    curT = util.transformation(rot, pos)
    ori = util.dcm2angle(rot)
    curVec = numpy.vstack([pos, ori])
    cur_d = getD(curVec, final, scale)
    print('cur_d:')
    print(cur_d)
    while (not rospy.is_shutdown() and cur_d > tolerance):
        J = rkin.jacobian()
        joint_states = rospy.wait_for_message("/robot/joint_states", JointState)
        cur_q = joint_states.position[9:16]
        cur_q = [cur_q[2], cur_q[3], cur_q[0], cur_q[1],
                 cur_q[4], cur_q[5], cur_q[6]]
        forward = rkin.forward_position_kinematics()
        quatangles = forward[3:]
        rot = util.quat2rot(quatangles[0], quatangles[1], quatangles[2],
                            quatangles[3])
        pos = numpy.matrix(forward[:3]).T
        print('pos:')
        print(pos)
        print('angles:')
        print(util.dcm2angle(rot))
        curT = util.transformation(rot, pos)
        # hand-rolled algorithm
        q_dot, out_q = util.resolvedRatesWithLimits(
            J, curT, final, speed, cur_q, freq, qMax, qMin, angular_tol)
        command_msg.command = out_q
        cur_d = getD(curVec, final, scale)
        print('cur_d:')
        print(cur_d)
        print('q_dot:')
        print(q_dot)
        print('cur_q:')
        print(cur_q)
        print('next q:')
        print(command_msg.command)
        print('command_msg:')
        print(command_msg)
        pub_joint_cmd.publish(command_msg)
        control_rate.sleep()
        ori = util.dcm2angle(rot)
        curVec = numpy.vstack([pos, ori])

joint_maxes = []
joint_mins = []

def main():
    print("Initializing node... ")
    rospy.init_node("examples")   # the node's name is examples
    print('initializing kinematics...')
    rkin = baxter_kinematics('right')
    print(rospy.wait_for_message('/robot/joint_states', JointState))
    return
    print('forward kinematics:')
    forward = rkin.forward_position_kinematics()
    print(forward)
    quatangles = forward[3:]
    print('angles:')
    rot = util.quat2rot(quatangles[0], quatangles[1], quatangles[2],
                        quatangles[3])
    print(rot)
    print('dcm2angle:')
    print(util.dcm2angle(rot))
    print('position:')
    pos = numpy.matrix(forward[:3]).T
    print(pos)
    print('transformation:')
    trans = util.transformation(rot, pos)
    print(trans)
    print('jocabian:')
    J = rkin.jacobian()
    print(J)
    # send_to_joint_vals([-numpy.pi/3,-numpy.pi/3,-numpy.pi/6,numpy.pi/5,0,-numpy.pi/2,0])
    final = numpy.matrix([[.8, 0, -.4,
                           0, 0, -numpy.pi/4]]).T
    speed = [2, .1]
    scale = [1, .2]
    runResolvedRates(rkin, final, speed, 100, scale, .05,
                     joint_maxes, joint_mins, 0.06)

if __name__ == '__main__':
    main()
