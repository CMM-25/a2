from constants import *
from utils import (
    derive_skew_ca,
    derive_rot_mat_ca,
    derive_homog_ca,
    derive_reverse_homog_ca,
    derive_mult_homog_point_ca,
    B_T_Bj,
    extract_state_np,
    extract_state_ca,
)
import numpy as np
import casadi as ca


def traj_opt(X_ref, U_ref, dt):
    skew_ca = derive_skew_ca()
    rot_mat_ca = derive_rot_mat_ca()
    homog_ca = derive_homog_ca()
    reverse_homog_ca = derive_reverse_homog_ca()
    mult_homog_point_ca = derive_mult_homog_point_ca()

    N = X_ref.shape[1]

    opti = ca.Opti()
    X = opti.variable(18, N)
    U = opti.variable(24, N)
    J = ca.MX(1, 1)

    for k in range(N):
        # extract state
        p, R, pdot, omega, p_j, f_j = extract_state_ca(X, U, k)
        if k != (N - 1):
            (
                p_next,
                R_next,
                pdot_next,
                omega_next,
                p_i_next,
                f_i_next,
            ) = extract_state_ca(X, U, k + 1)
        else:
            p_next, R_next, pdot_next, omega_next, p_i_next, f_i_next = (
                None,
                None,
                None,
                None,
                None,
                None,
            )

        # extract reference
        p_ref, R_ref, pdot_ref, omega_ref, p_i_ref, f_i_ref = extract_state_ca(
            X_ref, U_ref, k
        )

        # LQR weights
        Q_p = np.array([1000.0, 1000.0, 1000.0])
        Q_p_j = np.array([500.0, 500.0, 500.0])
        Q_pdot = np.array([10.0, 10.0, 10.0])
        Q_omega = np.array([1.0, 1.0, 1.0])
        Q_f_j = np.array([0.1, 0.1, 0.1])
        Q_R = np.eye(3) * 200.0

        # objective function
        # TODO: Ex.1 - Implement the objective function
        ##### only write your code here #####
        #                                   #
        #####################################

        # dynamics constraints
        # TODO: Ex.1 - Implement the dynamics constraints
        ##### only write your code here #####
        #                                   #
        #####################################

        # contact constraints
        # TODO: Ex.2 - Implement the contact constraints
        ##### only write your code here #####
        #                                   #
        #####################################

        # kinematics constraints
        # TODO: Ex.3 & 4 - Implement the kinematics constraints
        ##### only write your code here #####
        #                                   #
        #####################################

        f_lim = 20.0  # max vertical force in newtons
        mu = 0.9  # friction coefficient
        # friction pyramid constraints
        # TODO: Ex.5 - Implement the friction pyramid constraints
        ##### only write your code here #####
        #                                   #
        #####################################

    # apply objective function
    opti.minimize(J)

    # initial conditions constraint
    # TODO: Ex.1 - Implement the initial condition constraints
    ##### only write your code here #####
    #                                   #
    #####################################

    # initial solution guess
    opti.set_initial(X, X_ref)
    opti.set_initial(U, U_ref)

    # solve NLP
    p_opts = {}
    s_opts = {
        "print_level": 5,
        "max_iter": 500,
    }
    opti.solver("ipopt", p_opts, s_opts)
    sol = opti.solve()

    # extract solution as numpy array
    X_sol = np.array(sol.value(X))
    U_sol = np.array(sol.value(U))

    return X_sol, U_sol
