import matplotlib.pyplot as plt
import numpy as np
from math import ceil, floor, sqrt
import os

if __name__ == '__main__':
    # fl: np.ndarray = np.loadtxt('/tmp/fl.txt')
    # fr: np.ndarray = np.loadtxt('/tmp/fr.txt')
    cop: np.ndarray = np.loadtxt('/tmp/cop_computed.txt')
    zmp: np.ndarray = np.loadtxt('/tmp/mpc_zmp.txt')
    p_lsole: np.ndarray = np.loadtxt('/tmp/p_lsole.txt')
    p_rsole: np.ndarray = np.loadtxt('/tmp/p_rsole.txt')
    # alpha: np.ndarray = np.loadtxt('/tmp/alpha.txt')

    delta = 1e-3
    num_samples = cop.shape[0]
    t = np.linspace(0.0, delta * num_samples, num_samples)

    if not os.path.exists('images/mpc'):
        os.makedirs('images/mpc') 
    # transitions = delta * np.array(np.where(np.abs(np.diff(alpha)) > 0.05))

    # plt.figure()
    # plt.plot(t, fl)
    # plt.xlabel('Time [s]')
    # plt.ylabel('Force [N]')
    # plt.legend(['fx', 'fy', 'fz', 'taux', 'tauy', 'tauz'])
    # plt.title('Left Foot')
    # plt.grid()

    # plt.figure()
    # plt.plot(t, fr)
    # plt.xlabel('Time [s]')
    # plt.ylabel('Force [N]')
    # plt.legend(['fx', 'fy', 'fz', 'taux', 'tauy', 'tauz'])
    # plt.grid()
    # plt.title('Right Foot')

    fig = plt.figure()
    plt.plot(t, cop[:, 0])
    plt.plot(t, cop[:, 3])
    plt.plot(t, cop[:, 6])
    plt.plot(t, zmp[:, 0])
    plt.plot(t, p_lsole[:, 0])
    plt.plot(t, p_rsole[:, 0])
    y_lim = fig.axes[0].get_ylim()
    # for transition in transitions:
    #     plt.plot(np.array([transition, transition]), np.array([y_lim[0], y_lim[1]]), 'k--')
    # plt.plot(t, alpha)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend(['CoP Measured', 'CoP Filtered', 'ZMP Lip', 'ZMP', 'Left', 'Right'])
    plt.grid()
    plt.title('X CoP')

    fig.savefig(f"images/forces/X_CoP.png", bbox_inches='tight')

    fig = plt.figure()
    plt.plot(t, cop[:, 1])
    plt.plot(t, cop[:, 4])
    plt.plot(t, cop[:, 7])
    plt.plot(t, zmp[:, 1])
    plt.plot(t, p_lsole[:, 1])
    plt.plot(t, p_rsole[:, 1])
    y_lim = fig.axes[0].get_ylim()
    # for transition in transitions:
    #     plt.plot(np.array([transition, transition]), np.array([y_lim[0], y_lim[1]]), 'k--')
    # plt.plot(t, alpha)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend(['CoP Measured', 'CoP Filtered', 'ZMP Lip', 'ZMP', 'Left', 'Right'])
    plt.grid()
    plt.title('Y CoP')
    fig.savefig(f"images/forces/Y_CoP.png", bbox_inches='tight')

    fig = plt.figure()
    plt.plot(t, cop[:, 2])
    plt.plot(t, cop[:, 5])
    plt.plot(t, cop[:, 8])
    plt.plot(t, zmp[:, 2])
    plt.plot(t, p_lsole[:, 2])
    plt.plot(t, p_rsole[:, 2])
    y_lim = fig.axes[0].get_ylim()
    # for transition in transitions:
    #     plt.plot(np.array([transition, transition]), np.array([y_lim[0], y_lim[1]]), 'k--')
    # plt.plot(t, alpha)
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend(['CoP Measured', 'CoP Filtered', 'ZMP Lip', 'ZMP', 'Left', 'Right'])
    plt.grid()
    plt.title('Z CoP')
    fig.savefig(f"images/forces/Z_CoP.png", bbox_inches='tight')

# figs = []
    # for i in range(num_joints):
    #     if i % plots_per_fig == 0:
    #         figs.append(plt.figure())
    #     figs[-1].add_subplot(n, n, i % plots_per_fig + 1)
    #     plt.xlabel('Time [s]')
    #     plt.ylabel('Torque [Nm]')
    #     plt.plot(t, joint_eff[:, i])
    #     plt.title(joint_names[i])
    #     plt.grid()
    #     plt.tight_layout()

    # plt.show()