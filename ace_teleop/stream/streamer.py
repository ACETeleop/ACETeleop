import numpy as np

from ace_teleop.stream import handtracking_pb2, handtracking_pb2_grpc

from threading import Event
import time


class HandTrackingServicer(handtracking_pb2_grpc.HandTrackingServiceServicer):
    def __init__(self) -> None:
        self.update_event = Event()

        self.head = np.eye(4, 4)

        self.matrix_left = np.eye(4, 4)
        self.matrix_right = np.eye(4, 4)

        self.points_left = np.tile(np.ones(3), (25, 1))
        self.points_right = np.tile(np.ones(3), (25, 1))

        self.time = time.perf_counter()

    def StreamHandUpdates(self, request, context):

        while True:
            self.update_event.wait()

            self.update_event.clear()

            message = self.generate_hand_update()
            yield message

    def numpy_matrix_to_Matrix4x4(self, np_matrix):
        return handtracking_pb2.Matrix4x4(
            m00=np_matrix[0, 0],
            m01=np_matrix[0, 1],
            m02=np_matrix[0, 2],
            m03=np_matrix[0, 3],
            m10=np_matrix[1, 0],
            m11=np_matrix[1, 1],
            m12=np_matrix[1, 2],
            m13=np_matrix[1, 3],
            m20=np_matrix[2, 0],
            m21=np_matrix[2, 1],
            m22=np_matrix[2, 2],
            m23=np_matrix[2, 3],
            m30=np_matrix[3, 0],
            m31=np_matrix[3, 1],
            m32=np_matrix[3, 2],
            m33=np_matrix[3, 3],
        )

    def generate_joint_matrix(self, point):
        matrix = np.zeros((4, 4))
        # Set the rotation submatrix (top-left 3x3) to identity
        np.fill_diagonal(matrix, 1)
        # Set the translation (top-right 3x1)
        matrix[0:3, 3] = point
        # Ensure the last row is [0, 0, 0, 1] for homogeneous coordinates
        matrix[3, 3] = 1
        return matrix

    def points_to_skeleton(self, points):
        skeleton = handtracking_pb2.Skeleton()
        for point in points:
            matrix = self.generate_joint_matrix(point)
            # Convert the numpy matrix to Matrix4x4 format and add to skeleton
            matrix4x4 = self.numpy_matrix_to_Matrix4x4(
                matrix
            )  # Utilize the previous conversion function
            skeleton.jointMatrices.append(matrix4x4)
        return skeleton

    def generate_hand_update(self):
        # Example function to generate hand update data

        # print(self.matrix_right)
        hand_update = handtracking_pb2.HandUpdate(
            left_hand=handtracking_pb2.Hand(
                wristMatrix=self.numpy_matrix_to_Matrix4x4(self.matrix_left),
                skeleton=self.points_to_skeleton(self.points_left),
            ),
            right_hand=handtracking_pb2.Hand(
                wristMatrix=self.numpy_matrix_to_Matrix4x4(self.matrix_right),
                skeleton=self.points_to_skeleton(self.points_right),
            ),
            Head=self.numpy_matrix_to_Matrix4x4(self.head),
        )

        return hand_update
