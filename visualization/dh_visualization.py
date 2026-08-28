import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# AQUA-ARM DH FRAME VISUALIZATION
# ============================================================

DH = [
    [0.00,  np.pi / 2,  0.307, 0.0],   # J1
    [0.80,  0.0,       -0.020, 0.0],   # J2
    [0.00,  np.pi / 2,  0.350, 0.0],   # J3
    [0.13,  0.0,        0.920, 0.0],   # J4
]


def dh_matrix(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,       1]
    ])


def compute_frames(joint_angles):
    T = np.eye(4)
    frames = [T.copy()]

    for i in range(4):
        a, alpha, d, _ = DH[i]
        theta = joint_angles[i]

        T = T @ dh_matrix(a, alpha, d, theta)
        frames.append(T.copy())

    return frames


def draw_frame(ax, T, label):
    origin = T[:3, 3]

    x_axis = T[:3, 0]
    y_axis = T[:3, 1]
    z_axis = T[:3, 2]

    scale = 0.12

    ax.quiver(
        origin[0], origin[1], origin[2],
        x_axis[0], x_axis[1], x_axis[2],
        length=scale
    )

    ax.quiver(
        origin[0], origin[1], origin[2],
        y_axis[0], y_axis[1], y_axis[2],
        length=scale
    )

    ax.quiver(
        origin[0], origin[1], origin[2],
        z_axis[0], z_axis[1], z_axis[2],
        length=scale
    )

    ax.text(
        origin[0],
        origin[1],
        origin[2],
        label,
        fontsize=10
    )


def main():

    # Zero joint configuration
    joint_angles = np.array([
        0.0,
        0.0,
        0.0,
        0.0
    ])

    frames = compute_frames(joint_angles)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Draw coordinate frames
    for i, T in enumerate(frames):
        draw_frame(ax, T, f"Frame {i}")

    # Draw manipulator links
    positions = np.array([
        T[:3, 3]
        for T in frames
    ])

    ax.plot(
        positions[:, 0],
        positions[:, 1],
        positions[:, 2],
        marker="o",
        linewidth=2
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.set_title("AQUA-ARM DH Coordinate Frames")

    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()