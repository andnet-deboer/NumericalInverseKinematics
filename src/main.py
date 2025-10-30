"""Main entry point for the project."""

import matplotlib.pyplot as plt

# from matplotlib import cm
import numpy as np

import NewtonRaphsom as nr


def visualize(Tsb_history, lin_error, ang_error):
    """Visualize 3D convergence path and error reduction."""
    positions = np.array([T[:3, 3] for T in Tsb_history])
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]

    # Create 1x2 subplot grid
    fig = plt.figure(figsize=(10, 4))

    # Left: 3D scatter
    ax3 = fig.add_subplot(1, 2, 1, projection='3d')
    ax3.plot(x, y, z, label='parametric curve')
    ax3.set_xlabel('X-axis')
    ax3.set_ylabel('Y-axis')
    ax3.set_zlabel('Z-axis')
    ax3.set_title('End-Effector Trajectory (3D)')

    # # Right: 2D error plots
    fig, ax1 = plt.subplots(figsize=(6, 4))

    iterations = list(range(len(lin_error)))

    # Plot linear error on the left y-axis
    ax1.plot(iterations, lin_error, 'b-', label='Linear Error')
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Linear Error', color='b')
    ax1.tick_params(axis='y', labelcolor='b')

    # Create a twin y-axis for angular error on the right
    ax2 = ax1.twinx()
    ax2.plot(iterations, ang_error, 'r--', label='Angular Error')
    ax2.set_ylabel('Angular Error', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    # Add title and legends
    fig.suptitle('Error Convergence (2D)')
    ax1.legend(loc='upper right')
    ax2.legend(loc='upper center')

    plt.tight_layout()
    plt.show()


def main():
    """Run the main inverse kinematics demonstration."""
    L1, L2, W1, W2, H1, H2 = 425, 392, 109, 82, 89, 95

    M = np.array(
        [
            [-1, 0, 0, L1 + L2],
            [0, 0, 1, W1 + W2],
            [0, 1, 0, H1 - H2],
            [0, 0, 0, 1],
        ]
    )

    Blist = np.array(
        [
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -H1, 0, 0],
            [0, 1, 0, -H1, 0, L1],
            [0, 1, 0, -H1, 0, L1 + L2],
            [1, 0, 0, 0, W1 + W2, 0],
            [0, 1, 0, -(H1 - H2), 0, L1 + L2],
        ]
    ).T

    T_sd = np.array(
        [
            [0, 0, -1, 0],
            [1, 0, 0, 0.6],
            [0, -1, 0, 0],
            [0, 0, 0, 1],
        ]
    )

    long_iterates = np.array([1.350041, 0.631974, 1.944373, -1.317430,
                             -1.789641, 3.092790])
    short_iterates = np.array([0.010, -1.233, 0.887, 1.921, -3.132, -3.137])
    eomg = np.radians(0.057)
    ev = 0.0001

    iterations = [long_iterates, short_iterates]

    for i in iterations:
        # Run IK solver with iteration logging
        (
            thetalist,
            success,
            Tsb_history,
            lin_error_history,
            ang_error_history,
        ) = nr.IKinBodyIterates(
            M=M, Blist=Blist, T_sd=T_sd, thetalist0=i, eomg=eomg, ev=ev
        )

        print(f'\nFinal joint angles (radians): {np.round(thetalist, 4)}')
        print('Converged!' if success else 'Did not converge.')

        visualize(Tsb_history, lin_error_history, ang_error_history)


if __name__ == '__main__':
    main()
