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
robot_no = 14
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


if __name__=="__main__":
    # env = control_delta_arrays.DeltaArrayEnv("COM7")
    
    # wave = 0.5*np.sin(np.linspace(-2*np.pi, 2*np.pi, 20))
    wave = [1 for i in range(20)]
    # plt.plot(wave)
    # plt.show()
    
    delta_message = delta_trajectory_pb2.DeltaMessage()

    delta_message.id = robot_no
    delta_message.request_joint_pose = False
    delta_message.request_done_state = False
    delta_message.reset = False
    for i in range(10000):
        x = input("Go signal")
        # ee_pts = list(np.array(x.split(","), dtype=float))
        
        for j in range(10):
            ee_pts = [0,wave[j],10 + 2*wave[j]]
            pts = Delta.IK(ee_pts)
            pts = np.array(pts) * 0.01
            pts = np.clip(pts,0.005,0.095)
            jts = create_joint_positions(pts)
            _ = [delta_message.trajectory.append(jts[i]) for i in range(12)]
        a = 0.3
        for j in range(10,20):
            a *= -1
            ee_pts = [a*wave[j],0,10]
            pts = Delta.IK(ee_pts)
            pts = np.array(pts) * 0.01
            pts = np.clip(pts,0.005,0.095)
            jts = create_joint_positions(pts)
            _ = [delta_message.trajectory.append(jts[i]) for i in range(12)]

        serialized = delta_message.SerializeToString()
        
        print(len(serialized))
        print(len(delta_message.trajectory))
        # # print(serialized)
        # # if b"\xa7" in serialized:
        # #     print("HAKUNA")
            
        # # if b"n" in serialized:
        # #     print("HAKUNA")
            
        # # esp01.write(b'\xa6~~'+ serialized + b'\xa7~~\r\n')
        esp01.send(b'\xa6~~'+ serialized + b'\xa7~~\r\n')
        # del delta_message.trajectory[:]

        # done_moving = False
        # while not done_moving:
        #     print("HAKUNA")
        #     # time.sleep(10)
        #     # esp01.send(b'\xa6~~'+ serialized + b'\xa7~~\r\n')
        #     try:
        #         done_moving = esp01.recv(BUFFER_SIZE)
        #         print(done_moving.decode().strip())
        #     except:
        #         pass
        
        del delta_message.trajectory[:]


    # for i in np.arange(-2,2,0.1):
    #     for j in np.arange(-2,2,0.1):
    #         for k in np.arange(6,16,0.1):
    #             val = Delta.IK([i,j,k])
    #             val = np.array(val)/100
    #             vals = np.concatenate([val,val,val,val])
                
    #             for n in range(20):
    #                 _ = [delta_message.trajectory.append(vals[i]) for i in range(12)]
    #             serialized= delta_message.SerializeToString()
    #             del delta_message.trajectory[:]
    #             if b'p' in serialized:
    #                 print("HAKUNA")
    #                 # print(i,j,k)
    #                 print(len(serialized))
    #             else:
    #                 # print(serialized)
    #                 continue
    #                 # print(serialized)