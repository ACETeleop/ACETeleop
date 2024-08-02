import numpy as np

HEAD = np.array([[1, 0, 0, 0], [0, 0, -1, 0.0], [0, 1, 0, 1.5], [0, 0, 0, 1]])

YUP2ZUP = np.array(
    [[[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]], dtype=np.float64
)

MIRROR = np.array(
    [
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
)

YUP2ZUP_INV = np.transpose(YUP2ZUP, (0, 2, 1))
YUP2ZUP_INV_2D = YUP2ZUP_INV.reshape(4, 4)

# vision pro index
indices1 = [1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24]
# mediapipe index
indices2 = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

default_keypoint = np.array([0, 0, 0])

R_x_90_ccw_rot = np.array(
    [
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0],
    ]
)

R_y_90_ccw_rot = np.array(
    [
        [0, 0, 1],
        [0, 1, 0],
        [-1, 0, 0],
    ]
)

R_y_90_cw_rot = np.array(
    [
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0],
    ]
)

R_z_90_cw_rot = np.array(
    [
        [0, 1, 0],
        [-1, 0, 0],
        [0, 0, 1],
    ]
)

R_z_90_ccw_rot = np.array(
    [
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1],
    ]
)

R_z_90_ccw_pose = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
