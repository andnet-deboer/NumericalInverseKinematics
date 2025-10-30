"""Module to iterate through Newton-Raphson inverse kinematics."""

import csv

import modern_robotics as mr
import numpy as np


def IKinBodyIterates(
    Blist, M, T_sd, thetalist0, eomg, ev, csv_filename='iterates.csv'
):
    """
    Perform inverse kinematics using the Newton–Raphson method.

    Records and prints every iteration with detailed information.

    Args:
        Blist (ndarray): 6xn screw axes in body frame.
        M (ndarray): 4x4 home configuration of the end-effector.
        T_sd (ndarray): 4x4 desired end-effector configuration.
        thetalist0 (ndarray): Initial guess for joint angles.
        eomg (float): Angular error tolerance (radians).
        ev (float): Linear error tolerance (meters).
        csv_filename (str): Output filename for iteration log.

    Returns:
        tuple:
            thetalist (ndarray): Final joint angles.
            success (bool): True if converged.
            Tsb_history (list): List of SE(3) matrices per iteration.
            lin_error_history (list): Linear error magnitudes.
            ang_error_history (list): Angular error magnitudes.
    """
    thetalist = np.array(thetalist0, dtype=float)
    Tsb_history = []
    lin_error_history = []
    ang_error_history = []
    thetalist_history = []

    i = 0
    max_iterations = 50

    print('Starting IKinBodyIterates...\n')

    while i < max_iterations:

        # Space frame to body frame T_sb
        T_sb = mr.FKinBody(M, Blist, thetalist)

        # Desired configuratoin in end effector frame
        T_bd = np.dot(mr.TransInv(T_sb), T_sd)

        # Twist in {b} to get to {d}
        Vb = mr.se3ToVec(mr.MatrixLog6(T_bd))

        # Angular vector
        omega_b = Vb[0:3]

        # Linear vector
        v_b = Vb[3:6]

        err_omega = np.linalg.norm(omega_b)
        err_v = np.linalg.norm(v_b)

        Tsb_history.append(T_sb.copy())
        thetalist_history.append(thetalist.copy())
        lin_error_history.append(err_v)
        ang_error_history.append(err_omega)

        print(f'Iteration {i}:')
        print('joint vector:', ', '.join(f'{x:.3f}' for x in thetalist))
        print('SE(3) end-effector config:')
        print(np.round(T_sb, 3))
        print('error twist V_b:', np.round(Vb, 3))
        print(f'angular error ||omega_b||: {err_omega:.3f}')
        print(f'linear error ||v_b||: {err_v:.3f}\n')

        if err_omega < eomg and err_v < ev:
            success = True
            break

        Jb = mr.JacobianBody(Blist, thetalist)
        thetalist += np.dot(np.linalg.pinv(Jb), Vb)
        thetalist = np.arctan2(np.sin(thetalist), np.cos(thetalist))
        i += 1

    else:
        success = False

    # Save iteration log
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        for T in range(len(lin_error_history)):
            writer.writerow([f'{x:.6f}' for x in thetalist_history[T]])
        print(f'Saved iteration log to {csv_filename}')

    return (
        thetalist,
        success,
        Tsb_history,
        lin_error_history,
        ang_error_history,
    )
