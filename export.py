from scipy.spatial.transform import Rotation

from constants import *
from utils import extract_state_np, IK_np, jac_transpose_np


def export_to_csv(X, U, dt, fname):
    N = X.shape[1] - 1

    to_save = np.zeros((N + 1, 38))
    for k in range(N + 1):
        # extract state variables
        p, R, pdot, omega, p_j, f_j = extract_state_np(X, U, k)
        t = k * dt

        # convert orientation to quaternion
        quat_xyzw = Rotation.from_matrix(R).as_quat()
        quat = np.roll(quat_xyzw, 1)

        # negate quaternion if wrapping occurred compared to previous timestep
        if k > 0:
            for quat_idx in range(4):
                if abs(quat[quat_idx] - quat_prev[quat_idx]) > 1.0:
                    quat *= -1.0
                    continue

        # quaternion convention yf-08-18
        if k == 0:
            if quat[0] <= 0:
                quat *= -1
        else:
            for quat_idx in range(4):
                if abs(quat[quat_idx] - quat_prev[quat_idx]) > 1.0:
                    quat *= -1
                    continue

        # calculate joint angle q
        q_j = IK_np(p, R, p_j)
        q = []
        for leg in legs:
            q = np.hstack((q, q_j[leg]))

        # add or subtract 2*pi if wrapping occurred compared to previous timestep
        # (due to arctan2 in inverse kinematics)
        if k > 0 and np.linalg.norm(q - q_prev) > 3.0 / 2.0 * np.pi:
            for joint_idx in range(len(q)):
                if q[joint_idx] - q_prev[joint_idx] > 3.0 / 2.0 * np.pi:
                    q[joint_idx] -= 2 * np.pi
                if q[joint_idx] - q_prev[joint_idx] < -3.0 / 2.0 * np.pi:
                    q[joint_idx] += 2 * np.pi

        # calculate joint velocity qdot
        if k != 0:
            qdot = (q - q_prev) / dt
        else:
            qdot = np.zeros_like(q)

        # clip joint velocity, which is a hack to not make PD controller output crazy torques
        qdot_lim = 100.0  # joint velocity clip value in export()
        qdot = np.clip(qdot, -qdot_lim, qdot_lim)

        # calculate joint torque tau
        tau_j = jac_transpose_np(p, R, p_j, f_j)
        tau = []
        for leg in legs:
            tau = np.hstack((tau, tau_j[leg]))

        # note the reverse signs in joint variables to make it consistent
        # with RL and robot control code
        traj_t = np.hstack((t, p, quat, pdot, omega, -q, -qdot, -tau))
        to_save[k, :] = traj_t

        # store calculated q to be used for angular velocity calculation at next timestep
        q_prev = q
        quat_prev = quat

    np.savetxt("csv/" + fname + ".csv", to_save, delimiter=", ", fmt="%0.16f")
