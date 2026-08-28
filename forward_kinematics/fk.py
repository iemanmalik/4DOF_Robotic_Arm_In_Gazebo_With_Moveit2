import sys
from pathlib import Path

import numpy as np

# Allow importing dh_model when running this file directly
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dh_model.dh_parameters import DH_PARAMETERS, dh_transform


def forward_kinematics(q):
    """
    Calculate forward kinematics for the active J1-J4 chain.

    Parameters
    ----------
    q : array-like
        Joint angles [q1, q2, q3, q4] in radians.

    Returns
    -------
    T : numpy.ndarray
        Final 4x4 homogeneous transformation matrix.

    transforms : list
        Individual cumulative transforms.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError("q must contain exactly 4 joint angles.")

    T = np.eye(4)
    transforms = []

    for i in range(4):
        a, alpha, d, theta_offset = DH_PARAMETERS[i]

        A = dh_transform(
            a,
            alpha,
            d,
            q[i] + theta_offset
        )

        T = T @ A
        transforms.append(T.copy())

    return T, transforms


def end_effector_position(q):
    T, _ = forward_kinematics(q)
    return T[:3, 3]


def print_transform(T):
    np.set_printoptions(precision=5, suppress=True)
    print(T)


if __name__ == "__main__":

    # Test configuration
    q = np.array([
        0.0,
        0.0,
        0.0,
        0.0
    ])

    T, transforms = forward_kinematics(q)

    print("\nAQUA-ARM FORWARD KINEMATICS")
    print("=" * 50)

    print("\nJoint angles [rad]:")
    print(q)

    print("\nEnd-effector transformation T0_4:")
    print_transform(T)

    print("\nEnd-effector position [m]:")
    print(end_effector_position(q))

    print("\nIntermediate transformations:")

    for i, Ti in enumerate(transforms, start=1):
        print(f"\nT0_{i}:")
        print_transform(Ti)