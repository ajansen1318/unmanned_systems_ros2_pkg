from __future__ import annotations
from typing import Optional


class Node:
    """
    This is a simple 2D node to use the path finding algorithm

    """

    def __init__(
        self,
        x: float,
        y: float,
        cost: float = 0,
        parent_index: int = -1,
        parent_node: Optional[Node] = None,
    ) -> None:
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.parent_node = parent_node
