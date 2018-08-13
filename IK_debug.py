from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ##

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
        alpha0: 0,          a0: 0,      d1: d01,
        alpha1: -pi / 2,    a1: a12,    d2: 0,
        alpha2: 0,          a2: a23,    d3: 0,
        alpha3: -pi / 2,    a3: a34,    d4: d34,
        alpha4: pi / 2,     a4: 0,      d5: 0,
        alpha5: -pi / 2,    a5: 0,      d6: 0,
        alpha6: 0,          a6: 0,      d7: d67  # end effector
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

    T0_EE = reduce(lambda x, y: x * y, matrices)

    """
    Get the position of the end effector to calculate the actual position of the Wrist Center. 
    """

    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
        req.poses[x].orientation.x, req.poses[x].orientation.y,
        req.poses[x].orientation.z, req.poses[x].orientation.w])

    """
    Create the rotational transformation between the grip center and the endeffector to remove the rotational offset between both. 
    """

    r, p, y = symbols('r p y')

    ROT_x = Matrix([
        [1,      0,       0],
        [0, cos(r), -sin(r)],
        [0, sin(r), cos(r)]])  # ROLL

    ROT_y = Matrix([
        [cos(p),  0, sin(p)],
        [0,       1,      0],
        [-sin(p), 0, cos(p)]])  # PITCH

    ROT_z = Matrix([
        [cos(y), -sin(y), 0],
        [sin(y),  cos(y), 0],
        [0,            0, 1]])  # YAW

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

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3], FK[1,3], FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])

