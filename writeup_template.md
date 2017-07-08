## Project: Kinematics Pick & Place

---
**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # "Image References"

[DH_config]: ./misc_images/DH_config.jpg
[fk_demo]: ./misc_images/forward_demo.jpg
[DH]: ./misc_images/DH_config.png
[q23]: ./misc_images/q23.png
[IK_example]: ./misc_images/IK_example.png
[result]: ./misc_images/result.jpg

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Run `roslaunch kuka_arm forward_kinematics.launch` and screenshot it

![alt text][fk_demo]

We can find the URDF configuration in this forward kinematics demo or  `kr210.urdf.xacro`. Then we will figure out all parameters of the following DH figure and obtain Table.I.

![alt text][DH]

Table.I The relative location of joint i-1  to  i

| Joint   | X     | Y    | Z      | Roll, Pitch, Yaw |
| ------- | ----- | ---- | ------ | ---------------- |
| 1       | 0     | 0    | 0.33   | 0                |
| 2       | 0.35  | 0    | 0.42   | 0                |
| 3       | 0     | 0    | 1.25   | 0                |
| 4       | 0.96  | 0    | -0.054 | 0                |
| 5       | 0.54  | 0    | 0      | 0                |
| 6       | 0.193 | 0    | 0      | 0                |
| gripper | 0.11  | 0    | 0      | 0                |

Now, we can obtain our modified DH table.

Table. II The modified DH parameters

| Joint | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$    | $\theta_i$   |
| ----- | -------------- | --------- | -------- | ------------ |
| 1     | 0              | 0         | `0.75`   | `q1`         |
| 2     | `-pi/2`        | `0.35`    | 0        | `q2:q2-pi/2` |
| 3     | 0              | `1.25`    | 0        | `q3`         |
| 4     | `-pi/2`        | `-0.054`  | `1.5`    | `q4`         |
| 5     | `pi/2`         | 0         | 0        | `q5`         |
| 6     | `-pi/2`        | 0         | 0        | `q6`         |
| g     | 0              | 0         | `0.453` | `q7:0`       |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The DH parameter code
```
     s = {alpha0:       0, a0:      0, d1:  0.75,
          alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2,
          alpha2:       0, a2:   1.25, d3:     0,
          alpha3: -pi / 2, a3: -0.054, d4:   1.5,
          alpha4:  pi / 2, a4:      0, d5:     0,
          alpha5: -pi / 2, a5:      0, d6:     0,
          alpha6:       0, a6:      0, d7: 0.453, q7: 0}
```
The individual transformation matrix likes the following function `DH_transform()`.
```
    def DH_transform(q1, d1, alpha0, a0):
        T0_1 = Matrix([[cos(q1),     -sin(q1),          0,       a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), 
                     	  	     -sin(alpha0), 	 -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0),  
                    		      cos(alpha0),      cos(alpha0) * d1],
                       [      0,            0,          0,         1]])
        return T0_1
```

Then the homogeneous transform matrix about each joint can be derived using

$^{N}_0T = ^{0}_{1}T ^1_2T ^2_3T...^{N-1}_NT $

**Notice:** we need correct transform matrix from URDF frame to world frame multiplied by `R_corr` for the gripper pose.

```
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_7)
T_total = T0_G * R_corr
```

If we know the gripper pose, we can also obtain one transform matrix between `base_link` and `gripper_link`. 

Even though `roll, pitch, yaw` are terms in *intrinsic* rotation, the `roll, pitch, yaw` are calculated with `tf.transformations.euler_from_quaternion` with default rotation sequence from quaternion ('sxyz' is default setting) in this project code, hence *extrinsic* rotation is used here to calculate `Rrpy`.

```
Rrpy = rot_yaw(y)* rot_pitch(p) * rot_roll(r) * R_corr
```

##### Example:

To test our forward kinematics code implementation, we can set a random state in the above forward demo, use `thetas` in the slider window as inputs of our FK. Using the state in Fig.1 for example,

```
state = {q1: -1.06, q2:-0.54, q3:  0.94, 
         q4: -2.97, q5: 0.05, q6: -5.48}
print("T_total = ", T_total.evalf(subs=state))
```

The output is 

```
T_total =  Matrix([
[ 0.451648647906469,  -0.631020530156376, 0.630734959682294, 0.658912570897484],
[ -0.82338964238947, -0.0225645044787087, 0.567027635960868, -1.18106929707144],
[-0.343573857634714,  -0.775437898002668, -0.52976699660436,  1.08416816570233],
[                 0,                   0,                 0,               1.0]])
```

Execute gripper position from above matrix `p_fk = T_total[0:3,3]`, 

Find its position from RViz `p_g = Matrix([[0.65841], [-1.1795], [1.0837]])`.

We can calculate the FK error now. `error = p_fk - p_g` and `error.norm()`. 

This position error is `0.00171302362221487`.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

We have calculated our FK correctly, let's do inverse kinematics now.

1) Find the location of the WC relative to the base frame `pwc_0`. The wrist center is `joint 5`.

$pwc_0 = pg_0 - d \dot {}^0_6R \begin{bmatrix}0 \\ 0 \\ 1\end{bmatrix} =\begin{bmatrix}p_x\\p_y\\p_z\end{bmatrix} - d_G \dot{}^0_6R\begin{bmatrix}0 \\ 0 \\ 1\end{bmatrix}$

2) Find `theta1`

$\theta_1$ is quite straightforward, `theta1 = atan2(pwc_y, pwc_x`

![lt text][q23]

3) Find `theta3`

​	$\theta_{31} = atan2(a3, d4)$

​	$l_{35} = \sqrt{a_3^2 + d_4^2}$

​	$l_{25} = \sqrt{x_c^2 + y_c^2}$

​	$cos(\theta_{32}) = \frac{l_{25}^2 - a_2^2 - l_{35}^2}{2 \times a_2 \times l_{35} }$

​	$\theta_{32} = atan2(\sqrt{1- cos^2(\theta_{32})}, cos(\theta_{32}))$

​	$\theta_3 = \theta_{32} - \theta_{31} - \frac{\pi}{2}$	

**Note:** $x_c = \sqrt{pwc_x^2+pwc_y^2}$,  $ y_c = pwc_z$, `pwc` here is based on `joint2`. 

4) Find `theta2`

​	$\theta_{22} = atan2(y_c, x_c)$

​	$cos(\theta_{21}) = \frac{-l_{35}^2 + a_2^2 + l_{25}^2}{2 \times a_2 \times l_{25}}$

​	$\theta_{21} = atan2(\sqrt{1-cos^2(\theta_{21})}, cos(\theta_{21}))$

​	$\theta_2 = \frac{\pi}{2} - (\theta_{21} + \theta_{22})$

5) Find `theta4, 5 and 6`

The progress is quite straightforward. Because our URDF configuration doesn't follow DH conventions, the rotation matrix `R3_6` is not same with common matrices `yzy` , `zyz`, etc. But we have created all transformation matrices about `base frame` , includes `T0_3` and `T0_6`. Hence, it is easy to find `R3_6` symbol matrix for our project.

​	$^3_6R = ({^0_3T}^{-1} {}^0_6T)[:3,:3]$

```
R3_6 = 
([[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
  [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
  [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```

We can find `theta4 - 6` easily using the following equations.

    r12, r13 = R[0,1], R[0,2]
    r21, r22, r23 = R[1,0], R[1,1], R[1,2] 
    r32, r33 = R[2,1], R[2,2]
    # Euler angles from rotation matrix
    q5 = atan2(sqrt(r13**2 + r33**2), r23)
    q4 = atan2(r33, -r13)
    q6 = atan2(-r22, r21)
To solve the singularity problem, more conditional statements are used in code. 

##### Example:

Use the forward demo again, we obtain the gripper pose as input this time, and then we should compare our calculated results with those `thetas` in slider window. But it would be different on angle values because there are multiple solution for one gripper pose, hence, we use these calculated thetas as input our FK again to compare the FK's output with the gripper position from RViz here.![lt text][IK_example]

 As we can found in this figure, the gripper position is `[0.498,1.367,2.499]` and orientation is `[0.015,-0.185,0.939,0.288]`. 

1) Calculate the wrist center use 

```
pwc_0 = simplify(pg_0 - (0.303) * R0_g[0:3, 0:3]  * R_corr * Matrix([[0],[0],[1]]))
```
We can get `pwc_0` wrist position is

```
Matrix([[0.750499428337951],
        [ 1.20160389781975],
        [ 2.47518995758694]])
```

It is same with wrist center from RViz in above figure.

2) Calculate` theta1`
```
theta1 = atan2(pwc_0[1], pwc_0[0])
```

We got a different theta1 `theta1 = 1.0124980936377144` here.

3) Calculate `theta3`

```
p2_sym = T0_2 * Matrix([0,0,0,1])
p2_0 = p2_sym.evalf(subs={q1: theta1})
pwc_2 = pwc_0 - p2_0[0:3,:]
l23 = a2
l35 = sqrt(a3**2 + d4**2)
p25 = sqrt(np.sum(np.square(pwc_2)))
theta31 = atan2(a3,d4) # negative value
c235 = (np.sum(np.square(pwc_2)) - l23**2 - l35**2) / (2*l23*l35)
theta32 = atan2(sqrt(1-c235**2), c235)
theta3 = (theta32 + theta31 - pi/2).subs(s)
```

We got a different theta3 `theta3 = -0.11568665105374737` as well.

4) Calculate `theta2`

```
theta22 = atan2(pwc_2[2], sqrt(pwc_2[0]**2 + pwc_2[1]**2))
c523 = (-l35**2 + l23**2 + p25**2) / (2*l23 * p25)
theta21 = atan2(sqrt(1 - c523**2), c523)
theta2 = (pi/2 - (theta21 + theta22)).subs(s)
```

We got a similar theta2 with that from RViz `theta2 = -0.2758003637377226` finally...

5) Calculate `theta4-6`

```
def Euler_angles_from_matrix_URDF(R):
    r12, r13 = R[0,1], R[0,2]
    r21, r22, r23 = R[1,0], R[1,1], R[1,2] 
    r32, r33 = R[2,1], R[2,2]
    q5 = atan2(sqrt(r13**2 + r33**2), r23)
    q4 = atan2(r33, -r13)
    q6 = atan2(-r22, r21)
    return q4, q5, q6
R0_3 = T0_3[0:3, 0:3]
R0_3_inv = simplify(R0_3 ** -1)
R0_3_inv_value = R0_3_inv.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
R3_6 =  R0_3_inv_value * R0_g * R_corr
theta4, theta5, theta6 = Euler_angles_from_matrix_URDF(R3_6)
```
The `theta4-6` are calculated as following 
```
theta4 = 1.6344652724032285
theta5 = 1.5205000259943031
theta6 = -0.81578130619968381
```
We can find that the `theta5` is same with RViz, the others are not.
Substitute these values into our implemented FK code
```
state = {q1:	theta1, q2:	theta2, q3: theta3, 
         q4: 	theta4, q5:	theta5, q6:	theta6}
print("T_total = ", T_total.evalf(subs=state))
```

```
T_total =  Matrix([
[-0.833595473062543,  -0.48887208255965, 0.257143295038827, 0.49792],
[  0.54685182237706, -0.796053601157403, 0.259329420712765,  1.3673],
[0.0779209320563016,  0.356795110642117,  0.93092726749696,  2.4988],
[                 0,                  0,                 0,     1.0]])
```

`T_toal[:3,3]` is the gripper position from the inputs we calculated using IK. They are same with RViz. It shows our implementation is right!

We can implement our code into `IK_server.py` now!

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
![alt text][result]
1) I added `ros::Duration(5.0).sleep();` into `line 327` in the` /src/trajectory_sampler.cpp` following suggestions from @tiedyedguys in slack channel. 
without any comments in that code.

#### Postscript:

All the projects in the project are interesting but the course content is disappointing for a beginner.
Something that can be improved:
 Smooth the planning path, because the path generated by Moveit! is quite strange and in a weird curve. It's preferred if we can always find the shortest and smooth way to the target.

