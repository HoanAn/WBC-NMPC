#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import os

class Trajectory:
    def __init__(self, x, y, z, roll=None, pitch=None, yaw=None):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

# Read file composed of one position per line:
def read_position_file(path):
    # Init lists:
    xx = []
    yy = []
    zz = []

    # Read file:
    with open(path) as f:
        for l in f.readlines():
            p = l.rstrip('\n').split()
            xx.append(float(p[0]))
            yy.append(float(p[1]))
            zz.append(float(p[2]))

    # Setup np arrays:
    xx = np.array(xx)
    yy = np.array(yy)
    zz = np.array(zz)
    trajectory = Trajectory(xx, yy, zz)
    return trajectory

if __name__ == '__main__':
    mpc_com_file_path = '/tmp/mpc_com.txt'
    mpc_zmp_file_path = '/tmp/mpc_zmp.txt'
    com_file_path = '/tmp/com.txt'
    p_lsole_file_path = '/tmp/p_lsole.txt'
    p_rsole_file_path = '/tmp/p_rsole.txt'
    v_lsole_file_path = '/tmp/v_lsole.txt'
    v_rsole_file_path = '/tmp/v_rsole.txt'
    p_lsole_des_file_path = '/tmp/p_lsole_des.txt'
    p_rsole_des_file_path = '/tmp/p_rsole_des.txt'
    v_lsole_des_file_path = '/tmp/v_lsole_des.txt'
    v_rsole_des_file_path = '/tmp/v_rsole_des.txt'
    angular_momentum_file_path = '/tmp/angular_momentum.txt'

    mpc_com_trajectory = read_position_file(mpc_com_file_path)
    mpc_zmp_trajectory = read_position_file(mpc_zmp_file_path)
    com_trajectory = read_position_file(com_file_path)
    p_lsole_trajectory = read_position_file(p_lsole_file_path)
    p_rsole_trajectory = read_position_file(p_rsole_file_path)
    v_lsole_trajectory = read_position_file(v_lsole_file_path)
    v_rsole_trajectory = read_position_file(v_rsole_file_path)
    p_lsole_des_trajectory = read_position_file(p_lsole_des_file_path)
    p_rsole_des_trajectory = read_position_file(p_rsole_des_file_path)
    v_lsole_des_trajectory = read_position_file(v_lsole_des_file_path)
    v_rsole_des_trajectory = read_position_file(v_rsole_des_file_path)
    angular_momentum_trajectory = read_position_file(angular_momentum_file_path)

    delta_t = 1e-3
    samples = mpc_com_trajectory.x.shape[0]
    tt = np.linspace(0.0, delta_t * samples, samples)   

    if not os.path.exists('images/mpc'):
        os.makedirs('images/mpc') 

    sns.set(style='whitegrid')
    plt.rcParams.update({
        'font.size': 12,
        'lines.linewidth': 2,
        'axes.labelsize': 14,
        'axes.titlesize': 16,
        'legend.fontsize': 12,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12
    })

    # Combinazione di colori e marker
    ref_color = 'tab:red'
    actual_color = 'tab:green'
    zmp_color = 'tab:orange'

    #FIGURE: CoM and ZMP Trajectories
    fig, axs = plt.subplots(2, 2, figsize=(18, 10))
    fig.suptitle("MPC CoM and ZMP Trajectories", fontsize=18)

    ### 1. CoM/ZMP in XY ###
    ax1 = axs[0, 0]
    ax1.set_title("CoM and ZMP Trajectories (X-Y)")
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")

    ax1.plot(com_trajectory.x, com_trajectory.y, label='CoM', color=actual_color, marker='o', markersize=4)
    ax1.plot(mpc_com_trajectory.x, mpc_com_trajectory.y, label='CoM Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax1.plot(mpc_zmp_trajectory.x, mpc_zmp_trajectory.y, label='ZMP Ref', linestyle='-.', color=zmp_color)

    ax1.axis('equal')
    ax1.legend()
    ax1.grid(True)

    ### 2. CoM/ZMP X ###
    ax2 = axs[0, 1]
    ax2.set_title("X Trajectory Over Time")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("X [m]")

    ax2.plot(tt, com_trajectory.x, label='CoM.x', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax2.plot(tt, mpc_com_trajectory.x, label='CoM.x Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax2.plot(tt, mpc_zmp_trajectory.x, label='ZMP.x Ref', linestyle='-.', color=zmp_color)

    ax2.legend()
    ax2.grid(True)

    ### 3. CoM/ZMP Y ###
    ax3 = axs[1, 0]
    ax3.set_title("Y Trajectory Over Time")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Y [m]")

    ax3.plot(tt, com_trajectory.y, label='CoM.y', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax3.plot(tt, mpc_com_trajectory.y, label='CoM.y Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax3.plot(tt, mpc_zmp_trajectory.y, label='ZMP.y Ref', linestyle='-.', color=zmp_color)

    ax3.legend()
    ax3.grid(True)

    ### 4. CoM/ZMP Z ###
    ax4 = axs[1, 1]
    ax4.set_title("Z Trajectory Over Time")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Z [m]")

    ax4.plot(tt, com_trajectory.z, label='CoM.z', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax4.plot(tt, mpc_com_trajectory.z, label='CoM.z Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax4.plot(tt, mpc_zmp_trajectory.z, label='ZMP.z Ref', linestyle='-.', color=zmp_color)

    ax4.legend()
    ax4.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig("images/mpc/CoM_ZMP.png", dpi=300, bbox_inches='tight')



    #FIGURE: Left and Right Sole Positions

    fig_soles, axs = plt.subplots(3, 2, figsize=(18, 10))
    fig_soles.suptitle("Left and Right Sole Positions Over Time", fontsize=16)

    # Color/style settings
    des_style = {'linestyle': '--', 'linewidth': 2, 'alpha': 0.7}
    real_style = {'linestyle': '-', 'linewidth': 3, 'alpha': 0.7}

    # Left sole
    axs[0, 0].set_title("Left Sole - X")
    axs[0, 0].plot(tt, p_lsole_des_trajectory.x, label='Desired X', color='tab:blue', **des_style)
    axs[0, 0].plot(tt, p_lsole_trajectory.x, label='Actual X', color='tab:green', **real_style)
    axs[0, 0].set_xlabel('Time [s]')
    axs[0, 0].set_ylabel('X [m]')
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    axs[1, 0].set_title("Left Sole - Y")
    axs[1, 0].plot(tt, p_lsole_des_trajectory.y, label='Desired Y', color='tab:blue', **des_style)
    axs[1, 0].plot(tt, p_lsole_trajectory.y, label='Actual Y', color='tab:green', **real_style)
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Y [m]')
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    axs[2, 0].set_title("Left Sole - Z")
    axs[2, 0].plot(tt, p_lsole_des_trajectory.z, label='Desired Z', color='tab:blue', **des_style)
    axs[2, 0].plot(tt, p_lsole_trajectory.z, label='Actual Z', color='tab:green', **real_style)
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_ylabel('Z [m]')
    axs[2, 0].legend()
    axs[2, 0].grid(True)

    # Right sole
    axs[0, 1].set_title("Right Sole - X")
    axs[0, 1].plot(tt, p_rsole_des_trajectory.x, label='Desired X', color='tab:orange', **des_style)
    axs[0, 1].plot(tt, p_rsole_trajectory.x, label='Actual X', color='tab:red', **real_style)
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('X [m]')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    axs[1, 1].set_title("Right Sole - Y")
    axs[1, 1].plot(tt, p_rsole_des_trajectory.y, label='Desired Y', color='tab:orange', **des_style)
    axs[1, 1].plot(tt, p_rsole_trajectory.y, label='Actual Y', color='tab:red', **real_style)
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Y [m]')
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    axs[2, 1].set_title("Right Sole - Z")
    axs[2, 1].plot(tt, p_rsole_des_trajectory.z, label='Desired Z', color='tab:orange', **des_style)
    axs[2, 1].plot(tt, p_rsole_trajectory.z, label='Actual Z', color='tab:red', **real_style)
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel('Z [m]')
    axs[2, 1].legend()
    axs[2, 1].grid(True)

    # Final layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig_soles.savefig("images/mpc/soles_position.png", dpi=300, bbox_inches='tight')



    #FIGURE: CoM and ZMP Trajectories with Sole Positions (X + Y)

    fig, axs = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle("MPC CoM and ZMP Trajectories", fontsize=18)

    # Combinazione di colori e marker
    ref_color = 'tab:red'
    actual_color = 'tab:green'
    zmp_color = 'tab:orange'

    ### 2. CoM/ZMP X ###
    ax2 = axs[0]
    ax2.set_title("X Trajectory Over Time")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("X [m]")

    ax2.plot(tt, com_trajectory.x, label='CoM.x', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax2.plot(tt, mpc_com_trajectory.x, label='CoM.x Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax2.plot(tt, mpc_zmp_trajectory.x, label='ZMP.x Ref', linestyle='-.', color=zmp_color)
    ax2.plot(tt, p_lsole_trajectory.x, label='L Sole.x', linestyle=':', color='tab:blue')
    ax2.plot(tt, p_rsole_trajectory.x, label='R Sole.x', linestyle=':', color='tab:orange')
    ax2.plot(tt, p_lsole_des_trajectory.x, label='L Sole.x Ref', linestyle=':', color='tab:blue', alpha=0.5)
    ax2.plot(tt, p_rsole_des_trajectory.x, label='R Sole.x Ref', linestyle=':', color='tab:orange', alpha=0.5)

    ax2.legend()
    ax2.grid(True)

    ### 3. CoM/ZMP Y ###
    ax3 = axs[1]
    ax3.set_title("Y Trajectory Over Time")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Y [m]")

    ax3.plot(tt, com_trajectory.y, label='CoM.y', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax3.plot(tt, mpc_com_trajectory.y, label='CoM.y Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax3.plot(tt, mpc_zmp_trajectory.y, label='ZMP.y Ref', linestyle='-.', color=zmp_color)
    ax3.plot(tt, p_lsole_trajectory.y, label='L Sole.y', linestyle=':', color='tab:blue')
    ax3.plot(tt, p_rsole_trajectory.y, label='R Sole.y', linestyle=':', color='tab:orange')
    ax3.plot(tt, p_lsole_des_trajectory.y, label='L Sole.y Ref', linestyle=':', color='tab:blue', alpha=0.5)
    ax3.plot(tt, p_rsole_des_trajectory.y, label='R Sole.y Ref', linestyle=':', color='tab:orange', alpha=0.5)

    ax3.legend()
    ax3.grid(True)

    fig.savefig("images/mpc/CoM_ZMP_soles.png", dpi=300, bbox_inches='tight')


    

    #FIGURE: CoM and ZMP Trajectories with Sole Positions (XY + X + Y + Z)
    fig, axs = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle("MPC CoM and ZMP Trajectories", fontsize=18)

    # Combinazione di colori e marker
    ref_color = 'tab:red'
    actual_color = 'tab:green'
    zmp_color = 'tab:orange'

    ### 1. CoM/ZMP X ###
    ax2 = axs[0]
    ax2.set_title("X Trajectory Over Time")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("X [m]")

    ax2.plot(tt, com_trajectory.x, label='CoM.x', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax2.plot(tt, mpc_com_trajectory.x, label='CoM.x Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax2.plot(tt, mpc_zmp_trajectory.x, label='ZMP.x Ref', linestyle='-.', color=zmp_color)
    ax2.plot(tt, p_lsole_trajectory.x, label='L Sole.x', linestyle=':', color='tab:blue')
    ax2.plot(tt, p_rsole_trajectory.x, label='R Sole.x', linestyle=':', color='tab:orange')
    ax2.plot(tt, p_lsole_des_trajectory.x, label='L Sole.x Ref', linestyle=':', color='tab:blue', alpha=0.5)
    ax2.plot(tt, p_rsole_des_trajectory.x, label='R Sole.x Ref', linestyle=':', color='tab:orange', alpha=0.5)

    ax2.legend()
    ax2.grid(True)

    ### 2. CoM/ZMP Y ###
    ax3 = axs[1]
    ax3.set_title("Y Trajectory Over Time")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Y [m]")

    ax3.plot(tt, com_trajectory.y, label='CoM.y', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax3.plot(tt, mpc_com_trajectory.y, label='CoM.y Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax3.plot(tt, mpc_zmp_trajectory.y, label='ZMP.y Ref', linestyle='-.', color=zmp_color)
    ax3.plot(tt, p_lsole_trajectory.y, label='L Sole.y', linestyle=':', color='tab:blue')
    ax3.plot(tt, p_rsole_trajectory.y, label='R Sole.y', linestyle=':', color='tab:orange')
    ax3.plot(tt, p_lsole_des_trajectory.y, label='L Sole.y Ref', linestyle=':', color='tab:blue', alpha=0.5)
    ax3.plot(tt, p_rsole_des_trajectory.y, label='R Sole.y Ref', linestyle=':', color='tab:orange', alpha=0.5)

    ax3.legend()
    ax3.grid(True)

    ### 3. CoM/ZMP Z ###
    ax4 = axs[2]
    ax4.set_title("Z Trajectory Over Time")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Z [m]")

    ax4.plot(tt, com_trajectory.z, label='CoM.z', linestyle='-', color=actual_color, marker='o', markersize=4)
    ax4.plot(tt, mpc_com_trajectory.z, label='CoM.z Ref', linestyle='--', linewidth=3, alpha=0.6, color=ref_color)
    ax4.plot(tt, mpc_zmp_trajectory.z, label='ZMP.z Ref', linestyle='-.', color=zmp_color)
    ax4.plot(tt, p_lsole_trajectory.z, label='L Sole.z', linestyle=':', color='tab:blue')
    ax4.plot(tt, p_rsole_trajectory.z, label='R Sole.z', linestyle=':', color='tab:orange')
    ax4.plot(tt, p_lsole_des_trajectory.z, label='L Sole.z Ref', linestyle=':', color='tab:blue', alpha=0.5)
    ax4.plot(tt, p_rsole_des_trajectory.z, label='R Sole.z Ref', linestyle=':', color='tab:orange', alpha=0.5)

    ax4.legend()
    ax4.grid(True)

    # Final layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig("images/mpc/CoM_soles.png", dpi=300, bbox_inches='tight')



    #FIGURE: Left and Right Sole Velocities
    fig_v_soles, axs = plt.subplots(3, 2, figsize=(14, 9))
    fig_v_soles.suptitle("Left and Right Sole Velocities", fontsize=16)

    # Left sole velocity
    axs[0, 0].set_title("Left Sole Velocity - X")
    axs[0, 0].plot(tt, v_lsole_des_trajectory.x, label='Desired', color='tab:blue', **des_style)
    axs[0, 0].plot(tt, v_lsole_trajectory.x, label='Actual', color='tab:green', **real_style)
    axs[0, 0].set_xlabel('Time [s]')
    axs[0, 0].set_ylabel('X [m/s]')
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    axs[1, 0].set_title("Left Sole Velocity - Y")
    axs[1, 0].plot(tt, v_lsole_des_trajectory.y, label='Desired', color='tab:blue', **des_style)
    axs[1, 0].plot(tt, v_lsole_trajectory.y, label='Actual', color='tab:green', **real_style)
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Y [m/s]')
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    axs[2, 0].set_title("Left Sole Velocity - Z")
    axs[2, 0].plot(tt, v_lsole_des_trajectory.z, label='Desired', color='tab:blue', **des_style)
    axs[2, 0].plot(tt, v_lsole_trajectory.z, label='Actual', color='tab:green', **real_style)
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_ylabel('Z [m/s]')
    axs[2, 0].legend()
    axs[2, 0].grid(True)

    # Right sole velocity
    axs[0, 1].set_title("Right Sole Velocity - X")
    axs[0, 1].plot(tt, v_rsole_des_trajectory.x, label='Desired', color='tab:orange', **des_style)
    axs[0, 1].plot(tt, v_rsole_trajectory.x, label='Actual', color='tab:red', **real_style)
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('X [m/s]')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    axs[1, 1].set_title("Right Sole Velocity - Y")
    axs[1, 1].plot(tt, v_rsole_des_trajectory.y, label='Desired', color='tab:orange', **des_style)
    axs[1, 1].plot(tt, v_rsole_trajectory.y, label='Actual', color='tab:red', **real_style)
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Y [m/s]')
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    axs[2, 1].set_title("Right Sole Velocity - Z")
    axs[2, 1].plot(tt, v_rsole_des_trajectory.z, label='Desired', color='tab:orange', **des_style)
    axs[2, 1].plot(tt, v_rsole_trajectory.z, label='Actual', color='tab:red', **real_style)
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel('Z [m/s]')
    axs[2, 1].legend()
    axs[2, 1].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig_v_soles.savefig("images/mpc/soles_velocity.png", dpi=300, bbox_inches='tight')


    #FIGURE: Angular Momentum Trajectory    

    fig_ang_momentum, axs = plt.subplots(3, 1, figsize=(12, 7))
    fig_ang_momentum.suptitle("Angular Momentum Trajectory", fontsize=16)

    axs[0].set_title("Angular Momentum - X")
    axs[0].plot(tt, angular_momentum_trajectory.x, color='tab:purple', label='X')
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Momentum [kg·m²/s]')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].set_title("Angular Momentum - Y")
    axs[1].plot(tt, angular_momentum_trajectory.y, color='tab:cyan', label='Y')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Momentum [kg·m²/s]')
    axs[1].legend()
    axs[1].grid(True)

    axs[2].set_title("Angular Momentum - Z")
    axs[2].plot(tt, angular_momentum_trajectory.z, color='tab:brown', label='Z')
    axs[2].set_xlabel('Time [s]')
    axs[2].set_ylabel('Momentum [kg·m²/s]')
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig_ang_momentum.savefig("images/mpc/angular_momentum.png", dpi=300, bbox_inches='tight')