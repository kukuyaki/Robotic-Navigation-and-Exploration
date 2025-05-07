import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBasic(Controller):
    def __init__(self, kp=1, Lfc=10):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc

    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Search Front Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        Ld = self.kp*v + self.Lfc
        target_idx = min_idx
        for i in range(min_idx,len(self.path)-1):
            dist = np.sqrt((self.path[i+1,0]-x)**2 + (self.path[i+1,1]-y)**2)
            if dist > Ld:
                target_idx = i
                break
        target = self.path[target_idx]

        # TODO: Pure Pursuit Control for Basic Kinematic Model
        dx = target[0] - x
        dy = target[1] - y

        # 轉換為車體座標系下的座標
        fx = np.cos(yaw) * dx + np.sin(yaw) * dy
        fy = -np.sin(yaw) * dx + np.cos(yaw) * dy

        # 計算目標點與車體前進方向之間的夾角 alpha
        alpha = np.arctan2(fy, fx) - yaw

        # 計算轉彎半徑，然後用來求角速度 w
        Ld = self.kp*v +self.Lfc
        if Ld == 0:
            next_w = 0
        else:
            next_w = 2 * v * np.sin(alpha) / Ld  # Pure Pursuit 控制公式

        return next_w, target
