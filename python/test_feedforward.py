import numpy as np
import matplotlib.pyplot as plt
import time

from pyparsing import delimitedList
from Prismatic_Delta import Prismatic_Delta
from get_coords import RoboCoords
import delta_trajectory_pb2
from control_delta_arrays import DeltaArrayAgent
from serial_robot_mapping import delta_comm_dict, inv_delta_comm_dict

import telnetlib
import socket

# host = "192.168.1.31"
robot_no = 15
port = 80
timeout = 100
BUFFER_SIZE = 20

# esp01 = telnetlib.Telnet(host, port, timeout)
esp01 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
esp01.connect((inv_delta_comm_dict[robot_no], port))
esp01.settimeout(0.1)

# DDA = DeltaArrayAgent(esp01, delta_comm_dict[host])
DDA = DeltaArrayAgent(esp01, robot_no)
s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)

def create_joint_positions(val):
    a = []
    for i in range(4):
        for j in range(3):
            a.append(val[j])
    return a

def rotate(vector, angle, plot=False):
        # Rotation from Delta Array axis to cartesian axis = 30 degrees. 
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
        vector = vector@rot_matrix
        return vector

traj = [[0.0012993778983634763, -0.5235987755982988, 12.2001595082625],
#  [0.02431510295357648, -0.5235987755982988, 12.201572669850615],
#  [0.06893204933808428, -0.5235987755982988, 12.200691300958496],
#  [0.12952052727413474, -0.5235987755982988, 12.194693331215744],
#  [0.20229373935717573, -0.5235987755982988, 12.182002179493686],
#  [0.28424924572379173, -0.5235987755982988, 12.161551945387231],
 [0.37261725396096557, -0.5235987755982988, 12.132388570460279],
#  [0.46465505894317227, -0.5235987755982988, 12.093503949164198],
#  [0.5575945233829457, -0.5235987755982988, 12.043793733693589],
#  [0.6486388965486422, -0.5235987755982988, 11.98206980781886],
#  [0.7349787840405556, -0.5235987755982988, 11.90710060182216],
#  [0.8138317323634067, -0.5235987755982988, 11.817682744535857],
#  [0.882523740316103, -0.5235987755982988, 11.712764618326295],
 [0.9386325822351846, -0.5235987755982988, 11.591643976538865],
#  [0.9802039143078025, -0.5235987755982988, 11.454251659411957],
#  [1.0060356125564118, -0.5235987755982988, 11.30152024644078],
#  [1.0159882583156168, -0.5235987755982988, 11.135782130991075],
#  [1.0111924316483039, -0.5235987755982988, 10.960988514796776],
#  [0.9939720659460035, -0.5235987755982988, 10.782457510650541],
 [0.9674393419836608, -0.5235987755982988, 10.606114772087544],
 [0.9674393419836608, -0.5235987755982988, 10.606114772087544],
 [0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],[0.9674393419836608, -0.5235987755982988, 10.606114772087544],]

if __name__=="__main__":
    delta_message = delta_trajectory_pb2.DeltaMessage()
    delta_message.id = robot_no
    delta_message.request_joint_pose = False
    delta_message.request_done_state = False
    delta_message.reset = False
    for i in range(10000):
        x = input("Go signal")
        for j in range(20):
            # ee_pts = [0,wave[j],10 + 2*wave[j]]
            ee_pts = traj[j]
            pts = Delta.IK(ee_pts)
            pts = np.array(pts) * 0.01
            pts = np.clip(pts,0.005,0.095)
            jts = create_joint_positions(pts)
            _ = [delta_message.trajectory.append(jts[i]) for i in range(12)]

        serialized = delta_message.SerializeToString()
        
        print(len(serialized))
        print(len(delta_message.trajectory))

        esp01.send(b'\xa6~~'+ serialized + b'\xa7~~\r\n')
        del delta_message.trajectory[:]