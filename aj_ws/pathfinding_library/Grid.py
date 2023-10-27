from pathfinding_library.Obstacle import Obstacle

import matplotlib.pyplot as plt


class Grid:
    def __init__(
        self,
        min_x: int,
        min_y: int,
        max_x: int,
        max_y: int,
        grid_space: float,
        obstacles: [Obstacle],
        robot_radius: float = 0,
    ) -> None:
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.grid_space = grid_space
        self.obstacles = obstacles
        self.robot_radius = robot_radius

    def compute_index(
        self,
        x: int,
        y: int,
    ) -> int:
        index = int(
            ((self.max_x / self.grid_space) + 1) * (y / self.grid_space)
            + (x / self.grid_space)
        )  # row len * y + x

        return index

    def pos_in_obstacle(self, x_curr, y_curr) -> bool:
        # Check if near or inside obstacle
        for obs in self.obstacles:
            if obs.is_inside(x_curr, y_curr, self.robot_radius):
                return True
        return False

    def pos_outside_boundary(self, x_curr, y_curr) -> bool:
        # Check if outside boundary
        if self.min_x > x_curr:
            return True
        if self.max_x < x_curr:
            return True
        if self.min_y > y_curr:
            return True
        if self.max_y < y_curr:
            return True
        return False

    def pos_not_valid(
        self,
        x_curr: float,
        y_curr: float,
    ) -> bool:
        return self.pos_in_obstacle(x_curr, y_curr) or self.pos_outside_boundary(
            x_curr, y_curr
        )

    def plot(self, wp_list):
        # plot things
        # plt.figure(figsize=(7, 7))
        plt.xlim(self.min_x, self.max_x)
        plt.ylim(self.min_y, self.max_y)

        # plot obstacles
        for obstacle in self.obstacles:
            circle = plt.Circle(
                (obstacle.x_pos, obstacle.y_pos), obstacle.radius, color="r", fill=True
            )
            plt.gcf().gca().add_artist(circle)

        # plot path
        x_array = [wp[0] for wp in wp_list]
        y_array = [wp[1] for wp in wp_list]
        plt.plot(x_array, y_array)
        plt.show()
