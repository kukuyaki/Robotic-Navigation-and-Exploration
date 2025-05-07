import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBicycle(Controller):
    def __init__(self, kp=1, Lfc=25):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc

    # State: [x, y, yaw, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, v, l = info["x"], info["y"], info["yaw"], info["v"], info["l"]

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

                # TODO: Pure Pursuit Control for Bicycle Kinematic Model
        dx = target[0] - x
        dy = target[1] - y

        # 將目標點轉換到車體座標系
        fx = np.cos(yaw) * dx + np.sin(yaw) * dy
        fy = -np.sin(yaw) * dx + np.cos(yaw) * dy

        # 計算車體與目標點的夾角 alpha
        alpha = np.arctan2(fy, fx) - yaw

        # 計算實際前視距離
        Ld = self.kp*v +self.Lfc

        # 避免除以零
        if Ld == 0:
            next_delta = 0
        else:
            # Pure Pursuit 控制公式 for bicycle model
            next_delta = np.arctan2(2 * l * np.sin(alpha), Ld)

        return next_delta, target
