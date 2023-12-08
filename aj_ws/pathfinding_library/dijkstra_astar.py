import numpy as np
import math as m
import matplotlib.pyplot as plt

from pathfinding_library.node import Node
from pathfinding_library.Obstacle import Obstacle
from pathfinding_library.Grid import Grid


class DijkstraAstar:
    def __init__(
        self, grid: Grid, start_node: Node, end_node: Node, use_dijkstra: bool = False
    ) -> None:
        self.grid = grid
        self.start_node = start_node
        self.end_node = end_node
        self.use_dijkstra = use_dijkstra

        self.unvisited_nodes: dict[int, Node] = {}
        self.visited_nodes: dict[int, Node] = {}
        self.current_node = self.start_node
        self.current_index = self.grid.compute_index(
            self.current_node.x, self.current_node.y
        )

    def get_all_moves(self) -> [tuple]:
        gs_x_bounds = np.arange(
            -self.grid.grid_space,
            self.grid.grid_space + self.grid.grid_space,
            self.grid.grid_space,
        )
        gs_y_bounds = np.arange(
            -self.grid.grid_space,
            self.grid.grid_space + self.grid.grid_space,
            self.grid.grid_space,
        )
        move_list = []

        for dx in gs_x_bounds:
            for dy in gs_y_bounds:
                x_next = self.current_node.x + dx
                y_next = self.current_node.y + dy

                if [x_next, y_next] == [self.current_node.x, self.current_node.y]:
                    continue

                move = (x_next, y_next)
                move_list.append(move)
        return move_list

    def add_to_unvisited(self):
        # check if move is valid
        all_moves = self.get_all_moves()
        filtered_moves = []

        for move in all_moves:
            if self.grid.pos_not_valid(self.current_node.x, self.current_node.y):
                continue
            filtered_moves.append(move)

        for move in filtered_moves:
            new_index = self.grid.compute_index(move[0], move[1])
            # cost = parent cost + next distance
            if self.use_dijkstra == True:
                new_cost = self.current_node.cost + m.dist(
                    move, [self.current_node.x, self.current_node.y]
                )
            else:  # astar
                new_cost = self.current_node.cost + m.dist(
                    move, [self.current_node.x, self.current_node.y]
                )
                +m.dist(move, [self.end_node.x, self.end_node.y])
            if new_index in self.unvisited_nodes:
                if new_cost < self.unvisited_nodes[new_index].cost:
                    # update the cost value
                    self.unvisited_nodes[new_index].cost = new_cost
                    self.unvisited_nodes[new_index].parent_index = self.current_index
                continue

            elif new_index not in self.visited_nodes:
                new_node = Node(move[0], move[1], new_cost, self.current_index)
                self.unvisited_nodes[new_index] = new_node

    def find_path(self):
        # put current node in dictionary - use current_index as the key
        self.unvisited_nodes[self.current_index] = self.current_node
        while self.unvisited_nodes:
            self.current_index = min(
                self.unvisited_nodes, key=lambda x: self.unvisited_nodes[x].cost
            )
            self.current_node = self.unvisited_nodes[self.current_index]
            self.visited_nodes[self.current_index] = self.current_node

            del self.unvisited_nodes[self.current_index]

            self.add_to_unvisited()

        goal_node_index = self.grid.compute_index(self.end_node.x, self.end_node.y)
        wp_node = self.visited_nodes[goal_node_index]
        wp_list = []
        wp_list.append([wp_node.x, wp_node.y])

        while wp_node.parent_index != -1:
            next_index = wp_node.parent_index
            wp_node = self.visited_nodes[next_index]
            wp_list.append([wp_node.x, wp_node.y])

        return wp_list
