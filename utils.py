from constants import *
import numpy as np
import casadi as ca


# given 1x3 vector, returns 3x3 skew symmetric cross product matrix
def skew_np(s):
    return np.array([[0, -s[2], s[1]], [s[2], 0, -s[0]], [-s[1], s[0], 0]])


# derives a symbolic version of the skew function
def derive_skew_ca():
    s = ca.SX.sym("s", 3)

    skew_sym = ca.SX(3, 3)
    # skew_sym = ca.SX.zeros(3, 3)
    skew_sym[0, 1] = -s[2]
    skew_sym[0, 2] = s[1]
    skew_sym[1, 0] = s[2]
    skew_sym[1, 2] = -s[0]
    skew_sym[2, 0] = -s[1]
    skew_sym[2, 1] = s[0]

    return ca.Function("skew_ca", [s], [skew_sym])


# 2D rotation matrix
def rot_mat_2d_np(th):
    return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])


# given axis and angle, returns 3x3 rotation matrix
def rot_mat_np(s, th):
    # normalize s if isn't already normalized
    norm_s = np.linalg.norm(s)
    assert norm_s != 0.0
    s_normalized = s / norm_s

    # Rodrigues' rotation formula
    skew_s = skew_np(s_normalized)
    return np.eye(3) + np.sin(th) * skew_s + (1.0 - np.cos(th)) * skew_s @ skew_s


# derives a symbolic version of the rotMat function
def derive_rot_mat_ca():
    s = ca.SX.sym("s", 3)
    th = ca.SX.sym("th")
    skew_ca = derive_skew_ca()
    skew_sym = skew_ca(s)

    rot_mat_sym = (
            ca.SX.eye(3) + ca.sin(th) * skew_sym + (1 - ca.cos(th)) * skew_sym @ skew_sym
    )
    return ca.Function("rot_mat_ca", [s, th], [rot_mat_sym])


# given position vector and rotation matrix, returns 4x4 homogeneous
# transformation matrix
def homog_np(p, R):
    return np.block([[R, p[:, np.newaxis]], [np.zeros((1, 3)), 1]])


# position of corners of robot, in body frame (so it's a constant)
B_T_Bj = {}
for leg in legs:
    B_T_Bj[leg] = homog_np(B_p_Bj[leg], np.eye(3))


# derives a symbolic version of the homog function
def derive_homog_ca():
    p = ca.SX.sym("p", 3)
    R = ca.SX.sym("R", 3, 3)
    homog_sym = ca.SX(4, 4)
    homog_sym[:3, :3] = R
    homog_sym[:3, 3] = p
    homog_sym[3, 3] = 1.0
    return ca.Function("homog_ca", [p, R], [homog_sym])


# reverses the direction of the coordinate transformation defined by a 4x4
# homogeneous transformation matrix
def reverse_homog_np(T):
    R = T[:3, :3]
    p = T[:3, 3]
    reverse_homog = np.zeros((4, 4))
    reverse_homog[:3, :3] = R.T
    reverse_homog[:3, 3] = -R.T @ p
    reverse_homog[3, 3] = 1.0
    return reverse_homog


# derives a symbolic function that reverses the direction of the coordinate
# transformation defined by a 4x4 homogeneous transformation matrix
def derive_reverse_homog_ca():
    T = ca.SX.sym("T", 4, 4)
    R = T[:3, :3]
    p = T[:3, 3]
    reverse_homog_sym = ca.SX(4, 4)
    reverse_homog_sym[:3, :3] = R.T
    reverse_homog_sym[:3, 3] = -R.T @ p
    reverse_homog_sym[3, 3] = 1.0
    return ca.Function("reverse_homog_ca", [T], [reverse_homog_sym])


# multiplication between a 4x4 homogenous transformation matrix and 3x1
# position vector, returns 3x1 position
def mult_homog_point_np(T, p):
    p_aug = np.concatenate((p, [1.0]))
    return (T @ p_aug)[:3]


# derives a symbolic version of the mult_homog_point function
def derive_mult_homog_point_ca():
    T = ca.SX.sym("T", 4, 4)
    p = ca.SX.sym("p", 3)
    p_aug = ca.SX.ones(4, 1)
    p_aug[:3] = p
    mult_homog_point_sym = (T @ p_aug)[:3]
    return ca.Function("mult_homog_point_ca", [T, p], [mult_homog_point_sym])


# multiplication between a 4x4 homogenous transformation matrix and 3x1
# force vector, returns 3x1 force
def mult_homog_vec_np(T, f):
    f_aug = np.concatenate((f, [0.0]))
    return (T @ f_aug)[:3]


# generic planar 2 link inverse kinematics implementation
# returns the closest point within the workspace if the requested point is
# outside of it
def planar_IK_np(l1, l2, x, y):
    l = np.sqrt(x ** 2.0 + y ** 2.0)
    l = max(abs(l1 - l2), min(l, l1 + l2))

    alpha = np.arctan2(y, x)

    cos_beta = (l ** 2 + l1 ** 2 - l2 ** 2.0) / (2.0 * l * l1)
    cos_beta = max(-1.0, min(cos_beta, 1.0))
    beta = np.arccos(cos_beta)

    cos_th2_abs = (l ** 2 - l1 ** 2.0 - l2 ** 2.0) / (2.0 * l1 * l2)
    cos_th2_abs = max(-1.0, min(cos_th2_abs, 1.0))
    th2_abs = np.arccos(cos_th2_abs)

    th1 = alpha - beta
    th2 = th2_abs
    return th1, th2


# given numpy trajectory matrix, extract state at timestep k
# note the order argument in reshape, which is necessary to make it consistent
# with casadi's reshape
def extract_state_np(X, U, k):
    p = X[:3, k]
    R_flat = X[3:12, k]
    R = np.reshape(R_flat, (3, 3), order="F")
    pdot = X[12:15, k]
    omega = X[15:18, k]
    p_j = {}
    f_j = {}
    for leg in legs:
        p_j[leg] = U[3 * leg.value: leg.value * 3 + 3, k]
        f_j[leg] = U[12 + 3 * leg.value: 12 + leg.value * 3 + 3, k]
    return p, R, pdot, omega, p_j, f_j


# given casadi trajectory matrix, extract state at timestep k
def extract_state_ca(X, U, k):
    p = X[:3, k]
    R_flat = X[3:12, k]
    R = ca.reshape(R_flat, 3, 3)
    pdot = X[12:15, k]
    omega = X[15:18, k]
    p_j = {}
    f_j = {}
    for leg in legs:
        p_j[leg] = U[3 * leg.value: leg.value * 3 + 3, k]
        f_j[leg] = U[12 + 3 * leg.value: 12 + leg.value * 3 + 3, k]
    return p, R, pdot, omega, p_j, f_j


# given a numpy state, flattens it into the same form as a column of a
# trajectory matrix
def flatten_state_np(p, R, pdot, omega, p_j, f_j):
    R_flat = np.reshape(R, 9, order="F")
    p_j_flat = np.zeros(12)
    f_j_flat = np.zeros(12)
    for leg in legs:
        p_j_flat[3 * leg.value: leg.value * 3 + 3] = p_j[leg]
        f_j_flat[3 * leg.value: leg.value * 3 + 3] = f_j[leg]

    X_k = np.hstack((p, R_flat, pdot, omega))
    U_k = np.hstack((p_j_flat, f_j_flat))

    return X_k, U_k


def IK_np(p, R, p_j):
    T_B = homog_np(p, R)
    rotate_90 = rot_mat_2d_np(np.pi / 2.0)
    q_j = {}
    for leg in legs:
        T_Bj = T_B @ B_T_Bj[leg]
        Bj_T = reverse_homog_np(T_Bj)
        Bj_p_j = mult_homog_point_np(Bj_T, p_j[leg])
        # assert abs(Bj_p_j[1]) < eps # foot should be in shoulder plane
        x_z = rotate_90 @ np.array([Bj_p_j[0], Bj_p_j[2]])
        q1, q2 = planar_IK_np(l_thigh, l_calf, x_z[0], x_z[1])
        q_j[leg] = np.array([q1, q2])

    return q_j


def jac_transpose_np(p, R, p_j, f_j):

    def planar_jac_transpose_np(l1, l2, th1, th2, f1, f2):
        # TODO: Ex.10 - Implement the planar Jacobian transpose
        ##### only write your code here #####
        #                                   #
        tau = np.array([0.0, 0.0])
        #####################################
        return tau

    q_j = IK_np(p, R, p_j)
    T_B = homog_np(p, R)
    rotate_90 = rot_mat_2d_np(np.pi / 2.0)
    tau_j = {}
    for leg in legs:
        T_Bj = T_B @ B_T_Bj[leg]
        Bj_T = reverse_homog_np(T_Bj)
        # NOTE: ground reaction force needs to be negated to get force from robot to ground,
        # not from ground to robot
        Bi_f_i = mult_homog_vec_np(Bj_T, -f_j[leg])  # note negative sign
        # assert abs(Bi_f_i[1]) < eps # ground reaction force should be in shoulder plane
        f_xz = rotate_90 @ np.array([Bi_f_i[0], Bi_f_i[2]])
        tau_j[leg] = planar_jac_transpose_np(
            l_thigh, l_calf, q_j[leg][0], q_j[leg][1], f_xz[0], f_xz[1]
        )

    return tau_j
