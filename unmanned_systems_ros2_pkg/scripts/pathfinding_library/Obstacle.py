import numpy as np


class Obstacle:
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius

    def is_inside(self, curr_x: float, curr_y: float, robot_radius: float = 0) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos) ** 2 + (curr_y - self.y_pos) ** 2)
        if dist_from >= self.radius + robot_radius:
            return False

        return True

    def is_obstacle_in_path(current_position, target_position, obstacle_list):
        for obstacle in obstacle_list:
            if obstacle.is_inside(target_position[0], target_position[1]):
                return True  # obstacle detected
        return False
