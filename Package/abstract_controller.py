from abc import ABCMeta, abstractmethod
import numpy as np


class Controller(metaclass=ABCMeta):
    @abstractmethod
    def control(self, pts_2D, depth_array):
        pass

    @staticmethod
    def _calc_closest_dists_and_location(pose, pts_2D):
        location = np.array([pose.position.x_val, pose.position.y_val])
        # location = np.array([
        #     measurements.player_measurements.transform.location.x,
        #     measurements.player_measurements.transform.location.y,
        # ])
        dists = np.linalg.norm(pts_2D - location, axis=1)
        which_closest = np.argmin(dists)
        return which_closest, dists, location
