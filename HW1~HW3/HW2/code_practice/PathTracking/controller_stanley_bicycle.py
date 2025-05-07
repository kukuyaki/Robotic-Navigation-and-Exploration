import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, kp=0.5):
        self.path = None
        self.kp = kp

    # State: [x, y, yaw, delta, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v, l = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"]

        # Search Front Wheel Target
        front_x = x + l*np.cos(np.deg2rad(yaw))
        front_y = y + l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta))
        min_idx, min_dist = utils.search_nearest(self.path, (front_x,front_y))
        target = self.path[min_idx]

                # TODO: Stanley Control for Bicycle Kinematic Model

        # �����e����m�P���|�I��������V���]���|���u��V�^
        path_yaw = np.rad2deg(np.arctan2(
            self.path[min_idx+1,1] - self.path[min_idx,1],
            self.path[min_idx+1,0] - self.path[min_idx,0]
        )) if min_idx < len(self.path)-1 else yaw  # �̫�@�I�ɨ���W�X

        # Heading error
        heading_error = path_yaw - yaw
        heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

        # Cross Track error�]��V�~�t�^
        dx = target[0] - front_x
        dy = target[1] - front_y
        cross_track_error = dy * np.cos(np.deg2rad(path_yaw)) - dx * np.sin(np.deg2rad(path_yaw))

        # Stanley �����
        cross_term = np.rad2deg(np.arctan2(self.kp * cross_track_error, vf))
        next_delta = heading_error + cross_term

        # ������V���d��]�ھڹ�ڨ�������i�վ�^
        next_delta = np.clip(next_delta, -30.0, 30.0)

        return next_delta, target

