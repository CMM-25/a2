from constants import *
from utils import (
    rot_mat_2d_np,
    homog_np,
    mult_homog_point_np,
    B_T_Bj,
    extract_state_np,
    IK_np,
)
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

plt.style.use("seaborn-v0_8")


# draws a coordinate system defined by the 4x4 homogeneous transformation matrix T
def draw_T(T):
    axis_len = 0.1
    origin = T[:3, 3]
    axis_colors = ["r", "g", "b"]
    for axis in range(3):
        axis_head = origin + axis_len * T[:3, axis]
        axis_coords = np.vstack((origin, axis_head)).T

        line = plt.plot([], [])[0]
        line.set_data(axis_coords[0], axis_coords[1])
        line.set_3d_properties(axis_coords[2])
        line.set_color(axis_colors[axis])


def draw(p, R, p_j, f_j, f_len=0.02):
    T_B = homog_np(p, R)
    p_Bj = {}
    for leg in legs:
        p_Bj[leg] = mult_homog_point_np(T_B, B_p_Bj[leg])

    # draw body
    body_coords = np.vstack(
        (p_Bj[legs.FL], p_Bj[legs.FR], p_Bj[legs.HR], p_Bj[legs.HL], p_Bj[legs.FL])
    ).T
    line = plt.plot([], [])[0]
    line.set_data(body_coords[0], body_coords[1])
    line.set_3d_properties(body_coords[2])
    line.set_color("b")
    line.set_marker("o")

    # inverse and forward kinematics to extract knee location
    q_j = IK_np(p, R, p_j)
    p_knee_j = {}
    p_foot_j = {}
    for leg in legs:
        Bi_xz_knee = rot_mat_2d_np(q_j[leg][0] - np.pi / 2.0) @ np.array([l_thigh, 0.0])
        Bi_xz_foot = Bi_xz_knee + rot_mat_2d_np(
            q_j[leg][0] - np.pi / 2.0 + q_j[leg][1]
        ) @ np.array([l_calf, 0.0])
        Bi_p_knee_j = np.array([Bi_xz_knee[0], 0.0, Bi_xz_knee[1]])
        Bi_p_foot_j = np.array([Bi_xz_foot[0], 0.0, Bi_xz_foot[1]])
        T_Bi = T_B @ B_T_Bj[leg]
        p_knee_j[leg] = mult_homog_point_np(T_Bi, Bi_p_knee_j)
        p_foot_j[leg] = mult_homog_point_np(T_Bi, Bi_p_foot_j)

    # ensure foot positions match the values calculated from IK and FK
    # note that the y position of the legs are allowed to deviate from 0 by
    # amount eps in the kinematics constraint, so we use something larger here
    # to check if the error is "not close to zero"
    # for leg in legs:
    #     assert np.linalg.norm(p_foot_j[leg] - p_j[leg]) < np.sqrt(eps)

    # draw legs
    for leg in legs:
        leg_coords = np.vstack((p_Bj[leg], p_knee_j[leg], p_j[leg])).T
        line = plt.plot([], [])[0]
        line.set_data(leg_coords[0], leg_coords[1])
        line.set_3d_properties(leg_coords[2])
        line.set_color("g")
        line.set_marker("o")

    # draw ground reaction forces
    f_coords = {}
    for leg in legs:
        f_vec = p_j[leg] + f_len * f_j[leg]
        f_coords[leg] = np.vstack((p_j[leg], f_vec)).T
        line = plt.plot([], [])[0]
        line.set_data(f_coords[leg][0], f_coords[leg][1])
        line.set_3d_properties(f_coords[leg][2])
        line.set_color("r")

    draw_T(np.eye(4))
    draw_T(T_B)


def init_fig():
    anim_fig = plt.figure(figsize=(6, 6))
    ax = Axes3D(anim_fig, auto_add_to_figure=False)
    anim_fig.add_axes(ax)
    ax.view_init(azim=-45)
    ax.set_xlim3d([-0.5, 0.5])
    ax.set_ylim3d([-0.5, 0.5])
    ax.set_zlim3d([0, 1])
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    return anim_fig, ax


def animate_traj(X, U, dt, fname=None, display=True, repeat=False):
    anim_fig, ax = init_fig()

    def draw_frame(k):
        p, R, pdot, omega, p_j, f_j = extract_state_np(X, U, k)
        for line in ax.lines:
            line.remove()
        draw(p, R, p_j, f_j)

    N = X.shape[1] - 1

    anim = animation.FuncAnimation(
        anim_fig,
        draw_frame,
        frames=N + 1,
        interval=dt * 1000.0,
        repeat=repeat,
        blit=False,
    )

    if fname is not None:
        print("saving animation at videos/" + fname + ".mp4...")
        Writer = animation.writers["ffmpeg"]
        writer = Writer(fps=int(1 / dt), metadata=dict(artist="Me"), bitrate=1000)
        anim.save("videos/" + fname + ".mp4", writer=writer)
        print("finished saving videos/" + fname + ".mp4")

    if display:
        plt.show()
