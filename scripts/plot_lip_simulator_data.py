#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import numpy as np
import os

class LIPSimulatorLog:
    def __init__(self, lip_states=[], timings=[]):
        self.lip_states = lip_states
        self.timings = timings
    
    @staticmethod
    def initFromJson(lip_simulator_log_json):
        lip_states = []
        timings = []
        for data_iter in lip_simulator_log_json:
            lip_states.append(LIPState.initFromJson(data_iter["lip_state"]))
            timings.append(data_iter["timestep"])
        return LIPSimulatorLog(lip_states, timings)

    def extractTrajectories(self):
        com_pos_trajectory = []
        com_vel_trajectory = []
        zmp_pos_trajectory = []
        for lip_state in self.lip_states:
            com_pos_trajectory.append(lip_state.com_pos)
            com_vel_trajectory.append(lip_state.com_vel)
            zmp_pos_trajectory.append(lip_state.zmp_pos)
        com_pos_trajectory = np.array(com_pos_trajectory).T
        com_vel_trajectory = np.array(com_vel_trajectory).T
        zmp_pos_trajectory = np.array(zmp_pos_trajectory).T
        return np.array(self.timings), com_pos_trajectory, com_vel_trajectory, zmp_pos_trajectory

class LIPState:
    def __init__(self, com_pos, com_vel, zmp_pos):
        self.com_pos = com_pos
        self.com_vel = com_vel
        self.zmp_pos = zmp_pos
    
    @staticmethod
    def initFromJson(lip_state_json):
        com_pos_json = lip_state_json["com_pos"]
        com_pos = np.array([com_pos_json["x"], com_pos_json["y"], com_pos_json["z"]])
        com_vel_json = lip_state_json["com_vel"]
        com_vel = np.array([com_vel_json["x"], com_vel_json["y"], com_vel_json["z"]])
        zmp_pos_json = lip_state_json["zmp_pos"]
        zmp_pos = np.array([zmp_pos_json["x"], zmp_pos_json["y"], zmp_pos_json["z"]])
        return LIPState(com_pos, com_vel, zmp_pos)

if __name__ == '__main__':
    lip_simulator_data_file_path = "/tmp/labrob/lip_simulator/log.json"
    lip_simulator_data_json = json.load(open(lip_simulator_data_file_path))
    lip_simulator_log = LIPSimulatorLog.initFromJson(lip_simulator_data_json)
    timings, com_pos_trajectory, com_vel_trajectory, zmp_pos_trajectory = lip_simulator_log.extractTrajectories()

    print(timings.shape)
    print(com_pos_trajectory.shape)
    print(com_vel_trajectory.shape)
    print(zmp_pos_trajectory.shape)

    fig = plt.figure()

    ### Subplot CoM/ZMP ###
    ax1 = fig.add_subplot(4, 1, 1)
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')

    ax1.plot(zmp_pos_trajectory[0, :], zmp_pos_trajectory[1, :], label='ZMP')
    ax1.plot(com_pos_trajectory[0, :], com_pos_trajectory[1, :], label='CoM')

    ax1.legend()
    ax1.grid()
    ax1.axis('equal')

    ### Subplot ZMP.x ###
    ax2 = fig.add_subplot(4, 1, 2)
    ax2.set_xlabel('t')
    ax2.set_ylabel('x')

    ax2.plot(timings, zmp_pos_trajectory[0, :], label='ZMP.x')
    ax2.plot(timings, com_pos_trajectory[0, :], label='CoM.x')

    ax2.legend()
    ax2.grid()
    
    ### Subplot ZMP.y ###
    ax3 = fig.add_subplot(4, 1, 3)
    ax3.set_xlabel('t')
    ax3.set_ylabel('y')

    ax3.plot(timings, zmp_pos_trajectory[1, :], label='ZMP.y')
    ax3.plot(timings, com_pos_trajectory[1, :], label='CoM.y')

    ax3.legend()
    ax3.grid()

    ### Subplot ZMP.z ###
    ax4 = fig.add_subplot(4, 1, 4)
    ax4.set_xlabel('t')
    ax4.set_ylabel('y')

    ax4.plot(timings, zmp_pos_trajectory[2, :], label='ZMP.z')
    ax4.plot(timings, com_pos_trajectory[2, :], label='CoM.z')

    ax4.legend()
    ax4.grid()

    plt.show()