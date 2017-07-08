#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import tf2_ros
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, TransformStamped
from mpmath import *
from sympy import *
from sensor_msgs.msg import JointState
import numpy as np

def message_from_transform(T):
    msg = geometry_msgs.msg.Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    p = tf.transformations.translation_from_matrix(T)
    msg.translation.x = p[0]
    msg.translation.y = p[1]
    msg.translation.z = p[2]
    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

def homo_transformation(alpha, a, theta, d):

    rot_x   = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(alpha, 0.0, 0.0))
    trans_x = tf.transformations.translation_matrix([a, 0.0, 0.0])
    rot_z   = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, theta))
    trans_z = tf.transformations.translation_matrix([0.0, 0.0, d])
    
    return tf.transformations.concatenate_matrices(rot_x, trans_x, rot_z, trans_z)

class ik_calculator(object):
    
    def __init__(self):
        self.joint_values = [0.0 for i in range(7)]
        # joint state sub.
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        # ik calculator server
        self.ik_server = rospy.Service('calculate_ik', CalculateIK, self.handle_calculate_IK)
        # trans. publisher
        #self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # parameters
        self.params = {'alpha0':        0, 'a0':     0 , 'd1':  0.75,
                       'alpha1': -np.pi/2, 'a1':   0.35, 'd2':     0,
                       'alpha2':        0, 'a2':   1.25, 'd3':     0,
                       'alpha3': -np.pi/2, 'a3': -0.054, 'd4':   1.5,
                       'alpha4':  np.pi/2, 'a4':      0, 'd5':     0,
                       'alpha5': -np.pi/2, 'a5':      0, 'd6':     0,
                       'alpha6':        0, 'a6':      0, 'd7': 0.453}

    def FK_calculator(self,joint_values):
        
        s = {'alpha0':        0, 'a0':     0 , 'd1':  0.75, 'q1':           joint_values[0],
             'alpha1': -np.pi/2, 'a1':   0.35, 'd2':     0, 'q2': joint_values[1] - np.pi/2,
             'alpha2':        0, 'a2':   1.25, 'd3':     0, 'q3':           joint_values[2],
             'alpha3': -np.pi/2, 'a3': -0.054, 'd4':   1.5, 'q4':           joint_values[3],
             'alpha4':  np.pi/2, 'a4':      0, 'd5':     0, 'q5':           joint_values[4],
             'alpha5': -np.pi/2, 'a5':      0, 'd6':     0, 'q6':           joint_values[5],
             'alpha6':        0, 'a6':      0, 'd7': 0.453, 'q7':           joint_values[6]}
        
        # Define Modified DH Transformation matrix
        T01 = homo_transformation(s['alpha0'], s['a0'], s['q1'], s['d1'])
        T12 = homo_transformation(s['alpha1'], s['a1'], s['q2'], s['d2'])
        T23 = homo_transformation(s['alpha2'], s['a2'], s['q3'], s['d3'])
        T34 = homo_transformation(s['alpha3'], s['a3'], s['q4'], s['d4'])
        T45 = homo_transformation(s['alpha4'], s['a4'], s['q5'], s['d5'])
        T56 = homo_transformation(s['alpha5'], s['a5'], s['q6'], s['d6'])
        T6G = homo_transformation(s['alpha6'], s['a6'], s['q7'], s['d7'])

        # Create individual transformation matrices
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)
        T05 = np.dot(T04, T45)
        T06 = np.dot(T05, T56)
        T0G = np.dot(T06, T6G)

        # correcting transformations
        TC1 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, np.pi))
        TC2 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, -np.pi/2, 0.0))
        TC = np.dot(TC1,TC2)
        T0G = np.dot(T0G,TC)

        return T0G

    # joint state callback
    def joint_states_callback(self,joint_state):
        for i in range(2,len(joint_state.position)):
            self.joint_values[i - 2] = joint_state.position[i]

    def handle_calculate_IK(self,req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
            return -1
        else:
            # Initialize service response
            s = self.params
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
         
                # Calculate joint angles using Geometric IK method
                P0G = np.array([px, py, pz])
                R0G = tf.transformations.euler_matrix(roll, pitch, yaw)
                T0G = np.dot(tf.transformations.translation_matrix(P0G), tf.transformations.quaternion_matrix([req.poses[x].orientation.x, req.poses[x].orientation.y,
                        req.poses[x].orientation.z, req.poses[x].orientation.w]))
                
                ##########################################################################################
                # correcting transformations
                TC1 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, np.pi))
                TC2 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, -np.pi/2, 0.0))
                TC = np.dot(TC1,TC2)

                ############# theta 1 #############
                T0G_dh = np.dot(T0G,np.linalg.inv(TC))
                P = T0G_dh[:3,3]
                R = T0G_dh[:3, :3]

                ############# theta 1 #############
                P05 = P - s['d7']*R[:,2]
                theta1 = np.arctan2(P05[1],P05[0])

                
                ############ theta 2 ##########
                T01 = homo_transformation(s['alpha0'], s['a0'], theta1, s['d1'])
                T12 = homo_transformation(s['alpha1'], s['a1'], - np.pi/2, s['d2'])
                
                T02 = np.dot(T01, T12)
                P02 = T02[:3, 3]
                R02 = T02[:3,:3]

                T20 = np.linalg.inv(T02)
                R20 = np.linalg.inv(R02)
                
                # base_link
                P25 = P05 - P02
                
                # link_5 w.r.t frame 2
                P25_2 = np.dot(R20,P25)

                ### distance from joint 3 to 5
                d = np.sqrt(0.054**2 + (0.96 + 0.54)**2)
                # length of vector P25_2
                l = np.linalg.norm(P25_2)
                
                beta1 = np.arctan2(P25_2[0],P25_2[1])
                beta2 = np.arccos((l**2 + s['a2']**2 - d**2)/(2*s['a2']*l))

                theta2 = np.pi/2 - (beta1 + beta2)
                ##### theta 3 ######

                phi = np.arccos((s['a2']**2 + d**2 - l**2)/(2*s['a2']*d))
                alpha = np.arctan2(0.054,1.5)
                theta3 = np.pi/2 - (phi + alpha)

                ####### theta 4 #########
                #### no need to recalcolate
                #T01 = homo_transformation(s['alpha0'], s['a0'], theta1, s['d1'])
                T12 = homo_transformation(s['alpha1'], s['a1'], theta2 - np.pi/2, s['d2'])
                T23 = homo_transformation(s['alpha2'], s['a2'], theta3, s['d3'])
                T02 = np.dot(T01, T12)
                T03 = np.dot(T02, T23)
                
                R03 = T03[:3, :3]
                R36 = np.dot(np.linalg.inv(R03),R)

                theta4 = np.arctan2(R36[2,2],-R36[0,2]) 
                ######## theta 5 ######
                theta5 = np.arctan2(np.sqrt(R36[1,0]**2 + R36[1,1]**2),R36[1,2])
                ######## theta 6 ########
                theta6 = np.arctan2(-R36[1,1],R36[1,0])

                ## singularity
                if np.sin(theta5) < 0 :
                    theta4 = np.arctan2(-R36[2,2],R36[0,2]) 
                    theta6 = np.arctan2(R36[1,1],-R36[1,0])
                if np.allclose(theta5, 0):
                    theta4 = 0
                    theta6 = np.arctan2(-R36[0,1],-R36[2,1])

                #theta = [theta1, theta2, theta3, theta4, theta5, theta6, 0]


                # Populate response for the IK request
                # In the next line replace theta1,theta2...,theta6 by your joint angle variables
                joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
                joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)



if __name__ == "__main__":
    rospy.init_node('IK_server')
    ik_cal = ik_calculator()
    rospy.spin()
