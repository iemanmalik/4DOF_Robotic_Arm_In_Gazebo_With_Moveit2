import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from forward_kinematics.fk import forward_kinematics
from inverse_kinematics.ik import inverse_kinematics


def test_ik_fk_consistency():

    q_original = np.array([
        0.2,
        0.2,
        -0.2,
        -0.2
    ])

    target = forward_kinematics(
        q_original
    )[0][:3, 3]

    q_solution, success, _ = inverse_kinematics(
        target,
        initial_guess=np.zeros(4)
    )

    assert success

    final_position = forward_kinematics(
        q_solution
    )[0][:3, 3]

    error = np.linalg.norm(
        target - final_position
    )

    assert error < 1e-4


if __name__ == "__main__":

    test_ik_fk_consistency()

    print("IK/FK consistency test PASSED")