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
    command_msg.names=['right_e0', 'right_e1', 'right_s0', 'right_s1',
                       'right_w0', 'right_w1', 'right_w2']
    command_msg.command=q
    command_msg.mode=JointCommand.POSITION_MODE
    control_rate = rospy.Rate(100) # sending commands at 100HZ

    # acquire the current joint positions
    joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
    qc = joint_positions.position[9:16]

    print(command_msg)
    while not rospy.is_shutdown() and numpy.linalg.norm(numpy.subtract(q,qc))>0.01: # move until the desired joint variable
        pub_joint_cmd.publish(command_msg)    # sends the commanded joint values to Baxter
        control_rate.sleep()                    # sending commands at 100HZ
        joint_positions=rospy.wait_for_message("/robot/joint_states",JointState)
        qc = (joint_positions.position[9:16])
        #print "joint error = ", numpy.linalg.norm(numpy.subtract(q,qc))
    print("In home pose")
    return qc

def runResolvedRates(rkin, publish_fun, final, speed, freq, tolerance, alpha, qMax, qMin):
    util.resolvedRates(
        alpha, tolerance,
        lambda: rkin.jacobian(),
        rospy.Rate(freq),
        lambda: numpy.matrix([rospy.wait_for_message("/robot/joint_states", 
                                       JointState).position[9:16]]).T,
        lambda: getT(rkin),
        getPosOriVector,
        publish_fun,
        final,
        speed,
        1./freq)

def getT(rkin):
    forward = rkin.forward_position_kinematics()
    quatangles = forward[3:]
    rot = util.quat2rot(quatangles[0], quatangles[1], quatangles[2],
                        quatangles[3])
    pos = numpy.matrix(forward[:3]).T
    return util.transformation(rot, pos) 

# get position/orientation vector (6x1)
def getPosOriVector(T):
    rot = T[:3,:3]
    pos = T[:3, 3]
    ori = util.dcm2angle(rot)
    return numpy.vstack([pos, ori])

# ripped from baxter website
joint_maxes = [3.028, 2.618, .89, 1.047, 3.059, 2.094, 3.059]
joint_mins = [-3.028, -.052, -2.461, -2.147, -3.059, -1.571, -3.059]
middles = map(lambda (a, b): (a + b) / 2, zip(joint_maxes, joint_mins))

def main():
    print("Initializing node... ")
    rospy.init_node("examples")   # the node's name is examples
    msg = rospy.wait_for_message("/robot/joint_states", JointState)
    rkin = baxter_kinematics('right')
    ### top left
    # send_to_joint_vals([-numpy.pi/6,numpy.pi/5,-numpy.pi/3,-numpy.pi/3,0,numpy.pi/2,0])
    ### side
    # send_to_joint_vals(middles)
    ### front
    send_to_joint_vals([-0.117349530139,0, 1.01319430924, 0, 0.153398078613,0.626247655939,3.05108778362])
    ### desired
    # send_to_joint_vals([-0.117349530139,1.65017983068, 1.01319430924,-0.73784475813, 0.153398078613,0.626247655939,3.05108778362])
    final = numpy.matrix([[.67206115, -.16257794, .01516517,
                           3.12855761, -.03523214, .12796079]]).T

    pub_joint_cmd = rospy.Publisher(
        '/robot/limb/right/joint_command',JointCommand)
    command_msg = JointCommand()
    command_msg.names = ['right_e0', 'right_e1', 'right_s0', 'right_s1',
                         'right_w0', 'right_w1', 'right_w2']
    command_msg.mode=JointCommand.POSITION_MODE

    speed = 2
    alpha = .5
    freq = 100
    tolerance = .01
    runResolvedRates(
        rkin, 
        lambda q: publish_joints(pub_joint_cmd, command_msg, q),
        final, speed, freq, tolerance, alpha, joint_maxes, joint_mins)
    
# assumes that pub_joint_cmd and cmd_msg are appropriately constructed
def publish_joints(cmd, cmd_msg, q):
    cmd_msg.command = [float(q_i) for q_i in q]
    cmd.publish(cmd_msg)

if __name__ == '__main__':
    main()
