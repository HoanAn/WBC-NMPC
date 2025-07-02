import matplotlib.pyplot as plt
import numpy as np
from math import ceil, floor, sqrt
from collections import defaultdict
import os

if __name__ == '__main__':
    joint_vel: np.ndarray = np.loadtxt('/tmp/joint_vel.txt')
    joint_eff: np.ndarray = np.loadtxt('/tmp/joint_eff.txt')
    joint_names = open('/tmp/joint_names.txt').readlines()

    delta = 1e-3
    num_samples = joint_vel.shape[0]
    t = np.linspace(0.0, delta * num_samples, num_samples)

    if not os.path.exists('images/joints'):
        os.makedirs('images/joints')

    num_joints = joint_vel.shape[1]

    plots_per_fig = 4

    num_figs = ceil(num_joints / plots_per_fig)

    n = ceil(sqrt(plots_per_fig))

    figs = []
    # for i in range(num_joints):
    #     if i % plots_per_fig == 0:
    #         figs.append(plt.figure())
    #     figs[-1].add_subplot(n, n, i % plots_per_fig + 1)
    #     plt.xlabel('Time [s]')
    #     plt.ylabel('Angular Velocity [rad/s]')
    #     plt.plot(t, joint_vel_des[:, i], label='Desired')
    #     plt.plot(t, joint_vel[:, i], label='Actual')
    #     plt.title(joint_names[i])
    #     plt.legend()
    #     plt.grid()
    #     plt.tight_layout()

    figs = []
    for i in range(num_joints):
        if i % plots_per_fig == 0:
            figs.append(plt.figure())
        figs[-1].add_subplot(n, n, i % plots_per_fig + 1)
        plt.xlabel('Time [s]')
        plt.ylabel('Torque [Nm]')
        plt.plot(t, joint_eff[:, i])
        plt.title(joint_names[i])
        plt.grid()
        plt.tight_layout()

    grouped_indices = defaultdict(list)

    for idx, name in enumerate(joint_names):
        base_name = '_'.join(name.split('_')[:2])  # E.g., "left_ankle" da "left_ankle_roll_joint"
        grouped_indices[base_name].append(idx)
    
    #crea una directory per salvare le immagini se non esiste
    
    

    # Crea un plot per ogni gruppo
    figs = []
    for group_name, indices in grouped_indices.items():
        fig, ax = plt.subplots()
        for i in indices:
            ax.plot(t, joint_eff[:, i], label=joint_names[i])
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Torque [Nm]')
        ax.set_title(group_name.replace('_', ' ').title())
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        figs.append(fig)

        fig.savefig(f"images/joints/{group_name}_torque_plot.png")
        plt.close(fig)

    #plt.show()