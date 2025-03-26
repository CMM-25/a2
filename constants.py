import enum
import numpy as np


# enum for the four legs
class legs(enum.Enum):
    FL = 0
    FR = 1
    HL = 2
    HR = 3


# robot physical length parameters
l_Bx = 0.380  # length of body
l_By = 0.3  # width of body
l_thigh = 0.165  # length of upper leg
l_calf = 0.160  # length of lower leg

# robot inertial parameters
# mass of entire robot
m = 1.7
# moment of inertia of only the body
B_I = np.diag([0.00533767, 0.01314118, 0.01821833])
B_I_inv = np.diag(1 / np.array([0.00533767, 0.01314118, 0.01821833]))

# physical parameters external to robot
g = np.array([0.0, 0.0, -9.81])  # gravity vector

# position of corners of body, in body frame (so it's a constant)
B_p_Bj = {legs.FL: np.array([l_Bx / 2.0, l_By / 2.0, 0.0]),
          legs.FR: np.array([l_Bx / 2.0, -l_By / 2.0, 0.0]),
          legs.HL: np.array([-l_Bx / 2.0, l_By / 2.0, 0.0]),
          legs.HR: np.array([-l_Bx / 2.0, -l_By / 2.0, 0.0])}

# global optimization parameters
eps = 1e-6  # numerical zero threshold
