#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here

        print('Create parameters')

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        a12 = 0.35
        a23 = 1.25
        a34 = -0.054

        d01 = 0.75
        d34 = 1.5
        d67 = 0.303

        # Dictionary for constants
        constants = {
            alpha0: 0, a0: 0, d1: d01,
            alpha1: -pi / 2, a1: a12, d2: 0,
            alpha2: 0, a2: a23, d3: 0,
            alpha3: -pi / 2, a3: a34, d4: d34,
            alpha4: pi / 2, a4: 0, d5: 0,
            alpha5: -pi / 2, a5: 0, d6: 0,
            alpha6: 0, a6: 0, d7: d67  # end effector
        }

        class VariablePair:
            """
            Variable pairs to configure variables together with an offset.
            """

            def __init__(self, value, offset):
                self.value = value
                self.offset = offset

            def total(self):
                return self.value + self.offset

        class JointVariables:
            """
            All variables used in the homogeneous transformation for the joints.
            """

            def __init__(self, alpha, a, d, q):
                self.alpha = alpha
                self.a = a
                self.d = d
                self.q = q

        variables = [
            # T0_1
            JointVariables(
                alpha=VariablePair(alpha0, 0),
                a=VariablePair(a0, 0),
                d=VariablePair(d1, 0),
                q=VariablePair(q1, 0)
            ),
            # T1_2
            JointVariables(
                alpha=VariablePair(alpha1, 0),
                a=VariablePair(a1, 0),
                d=VariablePair(d2, 0),
                q=VariablePair(q2, - pi / 2)
            ),
            # T2_3
            JointVariables(
                alpha=VariablePair(alpha2, 0),
                a=VariablePair(a2, 0),
                d=VariablePair(d3, 0),
                q=VariablePair(q3, 0)
            ),
            # T3_4
            JointVariables(
                alpha=VariablePair(alpha3, 0),
                a=VariablePair(a3, 0),
                d=VariablePair(d4, 0),
                q=VariablePair(q4, 0)
            ),
            # T4_5
            JointVariables(
                alpha=VariablePair(alpha4, 0),
                a=VariablePair(a4, 0),
                d=VariablePair(d5, 0),
                q=VariablePair(q5, 0)
            ),
            # T5_6
            JointVariables(
                alpha=VariablePair(alpha5, 0),
                a=VariablePair(a5, 0),
                d=VariablePair(d6, 0),
                q=VariablePair(q6, 0)
            ),
            # T6_EE
            JointVariables(
                alpha=VariablePair(alpha6, 0),
                a=VariablePair(a6, 0),
                d=VariablePair(d7, 0),
                q=VariablePair(q7, 0)
            ),
        ]

        def createMatrix(variables):
            alpha = variables.alpha
            a = variables.a
            d = variables.d
            q = variables.q

            return Matrix([
                [cos(q.total()), -sin(q.total()), 0, a.total()],
                [sin(q.total()) * cos(alpha.total()), cos(q.total()) * cos(alpha.total()), -sin(alpha.total()),
                 -sin(alpha.total()) * d.total()],
                [sin(q.total()) * sin(alpha.total()), cos(q.total()) * sin(alpha.total()), cos(alpha.total()),
                 cos(alpha.total()) * d.total()],
                [0, 0, 0, 1]
            ])

        print('Create matrices')

        matrices = [createMatrix(variable) for variable in variables]
        matrices = [matrix.subs(constants) for matrix in matrices]

        print('Reduce to EE matrix')

        T0_1 = matrices[0]
        T1_2 = matrices[1]
        T2_3 = matrices[2]
        # T3_4 = matrices[3]
        # T4_5 = matrices[4]
        # T5_6 = matrices[5]
        # T6_7 = matrices[6]

        # Used for the FK in the debug file
        # T0_EE = reduce(lambda x, y: x * y, matrices)

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here

            """
            Create the rotational transformation between the grip center and the endeffector to remove the rotational offset between both. 
            """

            r, p, y = symbols('r p y')

            ROT_x = Matrix([
                [1, 0, 0],
                [0, cos(r), -sin(r)],
                [0, sin(r), cos(r)]])  # ROLL

            ROT_y = Matrix([
                [cos(p), 0, sin(p)],
                [0, 1, 0],
                [-sin(p), 0, cos(p)]])  # PITCH

            ROT_z = Matrix([
                [cos(y), -sin(y), 0],
                [sin(y), cos(y), 0],
                [0, 0, 1]])  # YAW

            ROT_EE = ROT_z * ROT_y * ROT_x

            """
            Correct the offset.
            """

            Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

            ROT_EE = ROT_EE * Rot_Error
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px], [py], [pz]])

            """
            To calculate the wrist center use the actual position of the endeffector and substract the distance between the 
            wrist center and the end effector in z direction.
            """
            WC = EE - (d67) * ROT_EE[:, 2]

            theta1 = atan2(WC[1], WC[0])

            r_wc = sqrt(WC[0] * WC[0] + WC[1] * WC[1])

            side_a = a23  # the vector magnitude between joint 2 and joint 3
            side_b = sqrt(pow(d34, 2) + pow(a34, 2))  # the vector magnitude between joint 3 and the WC
            side_c = sqrt(pow(r_wc - a12, 2) + pow((WC[2] - d01), 2))  # the vector magnitude between joint 2 and the WC

            # The angle between side_a and side_c
            beta_one = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            # The Angle between side_c and r_wc
            beta_two = atan2(WC[2] - d01, r_wc - a12)

            theta2 = pi / 2. - beta_one - beta_two

            # The angle between side_a and side_b
            beta_three = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
            # The angle between the z axis of joint 3 and its connection to the WC
            beta_four = acos((d34 * d34 + side_b * side_b - a34 * a34) / (2 * d34 * side_b))

            theta3 = pi / 2. - beta_three - beta_four

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Based on some discussions in the slack channel inverting the matrix for the inverse orientation kinematics can be
            # numerically unstable. That means there is no guarantee the process will converge.

            R3_6 = R0_3.transpose() * ROT_EE

            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
