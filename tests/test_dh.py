import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dh_model.dh_parameters import DH_PARAMETERS


def test_dh_parameters():

    assert len(DH_PARAMETERS) == 4

    expected = np.array([
        [0.00,  np.pi / 2,  0.307],
        [0.80,  0.00,      -0.020],
        [0.00,  np.pi / 2,  0.350],
        [0.13,  0.00,       0.920],
    ])

    actual = np.array([
        row[:3]
        for row in DH_PARAMETERS
    ])

    assert np.allclose(actual, expected)

    print("DH parameter test PASSED")


if __name__ == "__main__":
    test_dh_parameters()