import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, Q=np.eye(4), R=np.eye(1)):
        self.path = None
        self.Q = Q
        self.Q[0,0] = 1
        self.Q[1,1] = 1
        self.Q[2,2] = 1
        self.Q[3,3] = 1
        self.R = R*5000
        self.pe = 0
        self.pth_e = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    # State: [x, y, yaw, delta, v, l, dt]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v, l, dt = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"], info["dt"]
        yaw = utils.angle_norm(yaw)
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
                # TODO: LQR Control for Bicycle Kinematic Model

        # ���~�p��
        dx = target[0] - x
        dy = target[1] - y
        d_yaw = utils.angle_norm(target[2] - yaw)

        # �N�y�л~�t��쨮��y��
        e = np.cos(np.deg2rad(yaw)) * dy - np.sin(np.deg2rad(yaw)) * dx
        th_e = np.deg2rad(d_yaw)  # �ন���׵��᭱��

        # �~�t�ɼƻP�n��
        diff_e = (e - self.pth_e) / dt
        self.pe += e * dt
        self.pth_e = e

        # ���A�V�q�G[��V�~�t, ���׻~�t, ��V�~�t�ɼ�, ��V�~�t�n��]
        x_state = np.array([[e], [th_e], [diff_e], [self.pe]])

        # �u�ʤƨt�μҫ� A, B
        A = np.eye(4)
        A[0, 2] = dt
        A[0, 3] = 1.0
        A[1, 1] = 1.0
        A[1, 2] = dt
        A[2, 2] = 1.0
        A[3, 3] = 1.0

        B = np.zeros((4, 1))
        B[1, 0] = v / (l * (np.cos(np.deg2rad(delta)) ** 2)) * dt
        B[2, 0] = 1.0
        # �`�N B[3, 0] �O 0�A�]���n�����L���������J�v�T

        # �D�� LQR �̨μW�q K
        P = self._solve_DARE(A, B, self.Q, self.R)
        K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)

        # �p�ⱱ���X
        u = -K @ x_state
        next_delta = delta + np.rad2deg(u[0, 0])

        # ���� delta ���׽d��]�i�ھڨ�����O�վ�^
        next_delta = np.clip(next_delta, -30.0, 30.0)

        return next_delta, target

