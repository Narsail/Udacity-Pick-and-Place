Writeup: Kinematics Pick & Place
--- 

[//]: # (Image References)

[image1]: ./img/kuka_arm_schema.jpg
[image2]: ./img/inverse_position_kinematics.jpg

---
### 1. Deriving the Denavit-Hartenberg parameters from the robots structure

The following image contains the structure of the kuka_arm robot model. It was used to derive the necessary DH parameters
for the inverse kinematics calculations. As described in lesson 11 (chapter 18) the x-axes of joint 4 / 5 / 6 are centered
in the wrist center (joint 5). The mentioned joints form the wrist center of the robots gripper. 

![alt text][image1]

The following table contains the derived DH parameters from the drawing.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0,75 | q1
1->2 | - pi/2 | 0,35 | 0 | -pi/2 + q2
2->3 | 0 | 1,25 | 0 | q3
3->4 | - pi/2 | -0,054 | 1,5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0,303 | 0

### 2. Homogeneous transformation

By using the DH parameters of the above table we can create the homogeneous transformation matrix for every link. It describes the position and orientation of one joint reference to their linked joint.

The following method is used to create the transformation matrix:

```
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
```

To create all matrices we iterate over all variables and then sub the matrices with the constants from the DH parameters.

```
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
```

The constants are defined as follows:

```
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
```

The variables for each joint are wrapped into two classes to represent existing offsets:

```
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
```

The variables for each joint are defined as follows:

Joint 1 `T0_1`:
```
JointVariables(
    alpha=VariablePair(alpha0, 0),
    a=VariablePair(a0, 0),
    d=VariablePair(d1, 0),
    q=VariablePair(q1, 0)
)
```
Joint 2 `T1_2`:
```
JointVariables(
    alpha=VariablePair(alpha1, 0),
    a=VariablePair(a1, 0),
    d=VariablePair(d2, 0),
    q=VariablePair(q2, - pi / 2)
)
```
Joint 3 `T2_3`:
```
JointVariables(
    alpha=VariablePair(alpha2, 0),
    a=VariablePair(a2, 0),
    d=VariablePair(d3, 0),
    q=VariablePair(q3, 0)
)
```
Joint 4 `T3_4`:
```
JointVariables(
    alpha=VariablePair(alpha3, 0),
    a=VariablePair(a3, 0),
    d=VariablePair(d4, 0),
    q=VariablePair(q4, 0)
)
```

Joint 5 `T4_5`
```
JointVariables(
    alpha=VariablePair(alpha4, 0),
    a=VariablePair(a4, 0),
    d=VariablePair(d5, 0),
    q=VariablePair(q5, 0)
)
```

Joint 6 `T5_6`
```
JointVariables(
    alpha=VariablePair(alpha5, 0),
    a=VariablePair(a5, 0),
    d=VariablePair(d6, 0),
    q=VariablePair(q6, 0)
)
```
Joint 7 (End Effector) `T6_EE`
```
JointVariables(
    alpha=VariablePair(alpha6, 0),
    a=VariablePair(a6, 0),
    d=VariablePair(d7, 0),
    q=VariablePair(q7, 0)
)
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

With the help of the following drawings i derived the equations to calculate theta for joint 2 and 3. Calculating theta 1 was relatively simple after projecting the wrist center coordinates onto the x-y plane.

![alt text][image2]

```
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
```

The theta 4-6 values are calculated based on the rotation matrix of the end effector and the transposed homogeneous transformation matrix of joint 0 - 3. Transposing the matrix is a process discussed in the slack channel which saves compute time.

```
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

# Based on some discussions in the slack channel inverting the matrix for the inverse orientation kinematics can be
# numerically unstable. That means there is no guarantee the process will converge.

R3_6 = R0_3.transpose() * ROT_EE

theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

#### 4. Successful Pick and Place run

Video: https://vimeo.com/user83701479/review/285073001/bc68f52498

#### 5. Ways to improve the kuka arm

There are two ways (suggested by my first review) to further improve the performance of the code by

* using numpy instead of sympy 
* store the matrices so that they don't have to be created every time the IK_server.py will be reached

Additionally storing the fastest paths (with joint angles) for repetitive work might be worth checking.

