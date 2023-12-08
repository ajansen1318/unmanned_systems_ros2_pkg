import numpy as np
import math as m
import matplotlib.pyplot as plt
import random

from pathfinding_library.node import Node
from pathfinding_library.Obstacle import Obstacle
from pathfinding_library.Grid import Grid


class RRT:
    def __init__(
        self, grid: Grid, start_node: Node, end_node: Node, step_length: float
    ) -> None:
        self.grid = grid
        self.start_node = start_node
        self.end_node = end_node
        self.step_length = step_length

        self.tree: list[Node] = []
        self.current_node = self.start_node
        self.current_index = self.grid.compute_index(
            self.current_node.x, self.current_node.y
        )

    def find_path(self) -> [tuple]:
        self.tree.append(self.current_node)

        while (
            m.dist(
                [self.current_node.x, self.current_node.y],
                [self.end_node.x, self.end_node.y],
            )
            > self.step_length
        ):
            random_point = (
                random.uniform(self.grid.min_x, self.grid.max_x),
                random.uniform(self.grid.min_y, self.grid.max_y),
            )

            self.current_node = min(
                self.tree,
                key=lambda node: m.dist(
                    [node.x, node.y], [random_point[0], random_point[1]]
                ),
            )

            theta = m.atan2(
                (random_point[1] - self.current_node.y),
                random_point[0] - self.current_node.x,
            )

            new_point = (
                self.current_node.x + self.step_length * m.cos(theta),
                self.current_node.y + self.step_length * m.sin(theta),
            )
            if self.grid.pos_not_valid(new_point[0], new_point[1]):
                continue
            new_node = Node(
                x=new_point[0], y=new_point[1], parent_node=self.current_node
            )

            self.tree.append(new_node)
        for node in self.tree:
            plt.plot(node.x, node.y, "bo")
        plt.plot(0, 0, "go")
        plt.plot(9, 8, "ro")
        # plt.show()

        wp_node: Node = Node(
            x=self.end_node.x, y=self.end_node.y, parent_node=self.current_node
        )
        wp_list = []
        wp_list.append([wp_node.x, wp_node.y])

        while wp_node.parent_node:
            wp_node = wp_node.parent_node
            wp_list.append([wp_node.x, wp_node.y])

        return wp_list
