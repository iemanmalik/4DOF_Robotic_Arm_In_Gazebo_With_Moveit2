import numpy as np


# Standard DH parameters:
# [a, alpha, d, theta_offset]
#
# Active serial chain:
# J1 -> J2 -> J3 -> J4
#
# J5 is a mimic joint of J4 and is therefore not an
# independent serial DOF.

DH_PARAMETERS = np.array([
    [0.00,       np.pi / 2,  0.307,  0.0],   # J1
    [0.80,       0.0,       -0.02,   0.0],   # J2
    [0.00,       np.pi / 2,  0.35,   0.0],   # J3
    [0.13,       0.0,        0.92,   0.0],   # J4
], dtype=float)


def dh_transform(a, alpha, d, theta):
    """
    Standard Denavit-Hartenberg homogeneous transformation.

    Parameters
    ----------
    a : float
        Link length.
    alpha : float
        Link twist.
    d : float
        Link offset.
    theta : float
        Joint angle.

    Returns
    -------
    numpy.ndarray
        4x4 homogeneous transformation matrix.
    """

    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d],
        [0.0, 0.0,      0.0,     1.0]
    ])


def get_dh_table():
    return DH_PARAMETERS.copy()


if __name__ == "__main__":

    print("\nAQUA-ARM DH PARAMETERS")
    print("=" * 60)
    print("Joint       a(m)       alpha(rad)       d(m)       theta")
    print("-" * 60)

    for i, row in enumerate(DH_PARAMETERS, start=1):
        a, alpha, d, theta = row

        print(
            f"J{i:<8}"
            f"{a:<11.3f}"
            f"{alpha:<17.3f}"
            f"{d:<11.3f}"
            f"{theta:.3f}"
        )

    print("=" * 60)