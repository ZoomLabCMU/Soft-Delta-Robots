import numpy as np
import matplotlib.pyplot as plt
import time

from pyparsing import delimitedList
from Prismatic_Delta import Prismatic_Delta
from get_coords import RoboCoords
import delta_trajectory_pb2
from control_delta_arrays import DeltaArrayAgent
from serial_robot_mapping import delta_comm_dict, inv_delta_comm_dict
from math import sin, cos, tan, pi

import telnetlib
import socket

# host = "192.168.1.31"
robot_no = 15
port = 80
timeout = 100
BUFFER_SIZE = 20
NUM_MOTORS = 12

esp01 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
esp01.connect((inv_delta_comm_dict[robot_no], port))
esp01.settimeout(0.1)

# DDA = DeltaArrayAgent(esp01, delta_comm_dict[host])
DDA = DeltaArrayAgent(esp01, robot_no)
s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)

##### coordinate transforms ######
# H_a_b denotes homogeneous transform from frame b to frame a
R = 2.5
d1 = np.array([-R*cos(pi/3), -R*sin(pi/3), 0]).reshape((3,1))
d2 = np.array([R, 0, 0]).reshape((3,1))
d3 = np.array([-R*cos(pi/3), R*sin(pi/3), 0]).reshape((3,1))

H_w_1 = np.block([
    [np.eye(3), d1], 
    [np.zeros((1,3)), np.ones((1,1))]
])
H_w_2 = np.block([
    [np.eye(3), d2],
    [np.zeros((1,3)), np.ones((1,1))]
])
H_w_3 = np.block([
    [np.eye(3), d3],
    [np.zeros((1,3)), np.ones((1,1))]
])

H_1_w = np.block([
    [np.eye(3), -d1], 
    [np.zeros((1,3)), np.ones((1,1))]
])
H_2_w = np.block([
    [np.eye(3), -d2], 
    [np.zeros((1,3)), np.ones((1,1))]
])
H_3_w = np.block([
    [np.eye(3), -d3], 
    [np.zeros((1,3)), np.ones((1,1))]
])
#################  Traj  ############################
r_finger = 0.7 # cm
r_crush = 0.4 # cm to crush cup 
r_cup = 2.4 # cm

def cap_buffer(traj):
    n = len(traj)
    if n < BUFFER_SIZE:
        pts = traj[-1,:]
        tail = np.tile(pts, BUFFER_SIZE-n).reshape(-1, NUM_MOTORS)
        traj = np.vstack([traj, tail])
        return traj
    elif n == BUFFER_SIZE:
        return traj
    else:
        print("Trajectory exceeds buffer size, truncating...")
        traj = traj[0:BUFFER_SIZE, :]
        return traj

def get_start_points():
    x = 0
    y = 0
    z = 5.5
    t1 = np.array(np.tile([x,y,z], 4))
    return t1


def get_descent_points1():
    r = 2.4 #pillcup rad in cm
    r += 1 # "open hand"
    z1 = 10.5    
    z2 = 12.5
    p1_w = np.array([-r*cos(pi/3), -r*sin(pi/3), z1, 1]).reshape((4,1))
    p2_w = np.array([r, 0, z1, 1]).reshape((4,1))
    p3_w = np.array([-r*cos(pi/3), r*sin(pi/3), z1, 1]).reshape((4,1))
    
    p1_1 = (H_1_w @ p1_w)[0:3]
    p2_2 = (H_2_w @ p2_w)[0:3]
    p3_3 = (H_3_w @ p3_w)[0:3]
    p4 = [[0],[0],[5.5]]

    t1 = np.vstack([p1_1, p2_2, p3_3, p4])
    t1 = np.squeeze(t1)

    p1_w = np.array([-r*cos(pi/3), -r*sin(pi/3), z2, 1]).reshape((4,1))
    p2_w = np.array([r, 0, z2, 1]).reshape((4,1))
    p3_w = np.array([-r*cos(pi/3), r*sin(pi/3), z2, 1]).reshape((4,1))
    
    p1_1 = (H_1_w @ p1_w)[0:3]
    p2_2 = (H_2_w @ p2_w)[0:3]
    p3_3 = (H_3_w @ p3_w)[0:3]
    p4 = [[0],[0],[5.5]]

    t2 = np.vstack([p1_1, p2_2, p3_3, p4])
    t2 = np.squeeze(t2)

    t = np.vstack([t1, t2])
    return t

def get_grasp_points1():
    r = r_cup + r_finger - r_crush
    z = 12.5    
    th = pi/3
    p1_w = np.array([-r*cos(th), -r*sin(th), z, 1]).reshape((4,1))
    p2_w = np.array([r, 0, z, 1]).reshape((4,1))
    p3_w = np.array([-r*cos(th), r*sin(th), z, 1]).reshape((4,1))

    p1_1 = (H_1_w @ p1_w)[0:3]
    p2_2 = (H_2_w @ p2_w)[0:3]
    p3_3 = (H_3_w @ p3_w)[0:3]
    p4 = [[0],[0],[5.5]]

    t1 = np.vstack([p1_1, p2_2, p3_3, p4])
    t1 = np.squeeze(t1)
    return t1

def twistZ1(prev_state, deg):
    p1_1 = prev_state[0:3].reshape((3,1))
    p2_2 = prev_state[3:6].reshape((3,1))
    p3_3 = prev_state[6:9].reshape((3,1))

    #rotate the grasp points by 10 deg
    th = deg*pi/180
    p1_w = H_w_1 @ np.vstack([p1_1, [1]])
    p2_w = H_w_2 @ np.vstack([p2_2, [1]])
    p3_w = H_w_3 @ np.vstack([p3_3, [1]])

    R = np.array([[cos(th), -sin(th), 0, 0],
                  [sin(th), cos(th), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    
    p1r_w = R@p1_w
    p2r_w = R@p2_w
    p3r_w = R@p3_w


    p1r_1 = (H_1_w @ p1r_w)[0:3]
    p2r_2 = (H_2_w @ p2r_w)[0:3]
    p3r_3 = (H_3_w @ p3r_w)[0:3]
    p4 = [[0],[0],[5.5]]

    t1 = np.vstack([p1r_1, p2r_2, p3r_3, p4])
    t1 = np.squeeze(t1)
    return t1

def shake1(prev_state, d):
    p1_1 = prev_state[0:3].reshape((3,1))
    p2_2 = prev_state[3:6].reshape((3,1))
    p3_3 = prev_state[6:9].reshape((3,1))

    p1_w = H_w_1 @ np.vstack([p1_1, [1]])
    p2_w = H_w_2 @ np.vstack([p2_2, [1]])
    p3_w = H_w_3 @ np.vstack([p3_3, [1]])
    
    d = np.array([0, d, 0, 0]).reshape((4,1))

    p1d_w = p1_w + d
    p2d_w = p2_w + d
    p3d_w = p3_w + d


    p1d_1 = (H_1_w @ p1d_w)[0:3]
    p2d_2 = (H_2_w @ p2d_w)[0:3]
    p3d_3 = (H_3_w @ p3d_w)[0:3]
    p4 = [[0],[0],[5.5]]

    t1 = np.vstack([p1d_1, p2d_2, p3d_3, p4])
    t1 = np.squeeze(t1)
    return t1

def get_ascent_points1():
    #fudge factors often necessary to manage grasp closure with compliance
    r = r_cup + r_finger - r_crush - 0.2
    z = 7.5   
    th = pi/3
    p1_w = np.array([-r*cos(th), -r*sin(th), z, 1]).reshape((4,1))
    p2_w = np.array([r, 0, z, 1]).reshape((4,1))
    p3_w = np.array([-r*cos(th), r*sin(th), z, 1]).reshape((4,1))

    p1_1 = (H_1_w @ p1_w)[0:3]
    p2_2 = (H_2_w @ p2_w)[0:3]
    p3_3 = (H_3_w @ p3_w)[0:3]
    p4 = [[0],[0],[5.5]]

    t1 = np.vstack([p1_1, p2_2, p3_3, p4])
    t1 = np.squeeze(t1)
    return t1

def make_pillcup_traj1():
    # z range: (5cm, 12.5cm)
    t = []
    # start at top
    t_start = get_start_points()
    t.append(t_start)
    # descend to cup
    t_descent = get_descent_points1()
    t.append(t_descent)
    # grasp cup
    t_grasp = get_grasp_points1()
    t.append(t_grasp)
    # ascend
    t_ascent = get_ascent_points1()
    t.append(t_ascent)
    for _ in range(3):
        t_twistL = twistZ1(t_ascent, 20)
        t.extend([t_twistL, t_ascent])
    for _ in range(3):
        t_shook = shake1(t_ascent, 0.8)
        t.extend([t_shook, t_ascent])
    traj = np.vstack(t)
    traj = cap_buffer(traj)
    print(traj)
    traj[:,9] = np.zeros((BUFFER_SIZE,))
    traj[:,10] = np.zeros((BUFFER_SIZE,))
    traj[:,11] = 5.5*np.ones((BUFFER_SIZE,))
    return traj



if __name__=="__main__":
    traj = make_pillcup_traj1()
    jt_traj = np.zeros_like(traj)
    #print(traj)
    delta_message = delta_trajectory_pb2.DeltaMessage()
    delta_message.id = robot_no
    delta_message.request_joint_pose = False
    delta_message.request_done_state = False
    delta_message.reset = False
    for _ in range(10000):
        x = input("Go signal")
        # ee_pts = list(np.array(x.split(","), dtype=float))
        for i in range(BUFFER_SIZE):
            ee_pts = traj[i,:]
            jts = []
            for j in range(4):
                pts = Delta.IK(ee_pts[(3*j):(3*(j+1))])
                #print(pts)
                pts = np.array(pts) * 0.01
                pts = np.clip(pts, 0.005, 0.095)
                jts.extend(pts)
            jt_traj[i] = jts
            _ = [delta_message.trajectory.append(jts[i]) for i in range(12)]
        print(jt_traj)
        serialized = delta_message.SerializeToString()
        print(len(serialized))
        print(len(delta_message.trajectory))
        esp01.send(b'\xa6~~'+ serialized + b'\xa7~~\r\n')        
        del delta_message.trajectory[:]