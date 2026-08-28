import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from forward_kinematics.fk import forward_kinematics


# Joint limits [rad]
JOINT_LIMITS = np.array([
    [-np.pi / 2, np.pi / 2],   # J1
    [-np.pi / 2, np.pi / 2],   # J2
    [-np.pi / 2, np.pi / 2],   # J3
    [-np.pi / 2, 0.0],          # J4
])


def numerical_jacobian(q, delta=1e-6):
    q = np.asarray(q, dtype=float)

    J = np.zeros((3, 4))

    p0 = forward_kinematics(q)[0][:3, 3]

    for i in range(4):
        q_delta = q.copy()
        q_delta[i] += delta

        p_delta = forward_kinematics(q_delta)[0][:3, 3]

        J[:, i] = (p_delta - p0) / delta

    return J


def inverse_kinematics(
    target_position,
    initial_guess=None,
    max_iterations=1000,
    tolerance=1e-5,
    step_size=0.5
):
    target = np.asarray(target_position, dtype=float)

    if target.shape != (3,):
        raise ValueError(
            "target_position must contain [x, y, z]"
        )

    if initial_guess is None:
        q = np.zeros(4)
    else:
        q = np.asarray(initial_guess, dtype=float).copy()

    for iteration in range(max_iterations):

        current = forward_kinematics(q)[0][:3, 3]

        error = target - current

        if np.linalg.norm(error) < tolerance:
            return q, True, iteration

        J = numerical_jacobian(q)

        # Damped least-squares inverse
        damping = 1e-4

        J_inv = (
            J.T
            @ np.linalg.inv(
                J @ J.T +
                damping * np.eye(3)
            )
        )

        dq = step_size * J_inv @ error

        q += dq

        # Apply joint limits
        for i in range(4):
            q[i] = np.clip(
                q[i],
                JOINT_LIMITS[i, 0],
                JOINT_LIMITS[i, 1]
            )

    return q, False, max_iterations


if __name__ == "__main__":

    print("\nAQUA-ARM INVERSE KINEMATICS")
    print("=" * 50)

    # Desired XYZ position
    target = np.array([
        0.95372321,
        -0.11703025,
        -0.45406454
    ])

    q_solution, success, iterations = inverse_kinematics(
        target,
        initial_guess=np.array([
            0.0,
            0.0,
            0.0,
            0.0
        ])
    )

    print("\nTarget position:")
    print(target)

    print("\nIK solution [rad]:")
    print(q_solution)

    print("\nIK solution [deg]:")
    print(np.degrees(q_solution))

    print("\nSuccess:")
    print(success)

    print("\nIterations:")
    print(iterations)

    # Verify using FK
    final_position = forward_kinematics(
        q_solution
    )[0][:3, 3]

    print("\nFK verification position:")
    print(final_position)

    print("\nPosition error:")
    print(np.linalg.norm(target - final_position))