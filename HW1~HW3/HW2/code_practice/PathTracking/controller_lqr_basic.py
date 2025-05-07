import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBasic(Controller):
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
        x, y, yaw, v, dt = info["x"], info["y"], info["yaw"], info["v"], info["dt"]
        yaw = utils.angle_norm(yaw)
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
                # TODO: LQR Control for Basic Kinematic Model

        # 誤差計算
        dx = target[0] - x
        dy = target[1] - y
        d_yaw = utils.angle_norm(target[2] - yaw)

        # 錯誤轉換到車體座標系
        e = np.cos(yaw) * dy - np.sin(yaw) * dx
        th_e = d_yaw

        # 差分狀態定義
        self.pe += e * dt
        diff_e = (e - self.pth_e) / dt
        self.pth_e = e

        # 狀態向量: [e, th_e, de, ∫e]
        x_state = np.array([[e], [th_e], [diff_e], [self.pe]])

        # 線性化系統動態（以基本模型推導）
        A = np.eye(4)
        A[0, 2] = dt
        A[0, 3] = 1.0
        A[1, 1] = 1.0
        A[1, 2] = dt
        A[2, 2] = 1.0
        A[3, 3] = 1.0

        B = np.zeros((4,1))
        B[2,0] = 1.0
        B[3,0] = 0.0  # 積分項不受控制輸入影響

        # 求解 LQR 最佳增益 K
        P = self._solve_DARE(A, B, self.Q, self.R)
        K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)

        # 控制器輸出
        u = -K @ x_state
        next_w = u[0,0]

        return next_w, target
