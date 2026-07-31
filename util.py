from math import sqrt

import networkx as nx
import osmnx as ox
from networkx.classes.multidigraph import MultiDiGraph


class Util:

    def canReachGoal(self, G: MultiDiGraph, start_node, goal_node):
        return goal_node in nx.descendants(G, start_node)

    @staticmethod
    def _reconstruct_path(came_from, current):
        path = []
        node = current
        while node in came_from:
            predecessor, edge_key = came_from[node]
            path.append((node, edge_key))
            node = predecessor
        path.append((node, None))   # start node: no incoming edge
        path.reverse()
        return path

    @staticmethod
    def straight_line_distance(G, u, v):
        # Euclidean distance between two nodes' projected coordinates, in
        # the graph's projected units (meters after ox.projection.project_graph).
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @classmethod
    def speed_and_distance_heuristic(cls, G, max_speed_mps):
        """
        Returns a heuristic(node, goal) closure admissible for travel_time_weight:
        straight-line distance to the goal divided by the fastest possible
        speed anywhere in the graph. Since no real route can be faster than
        max_speed_mps, this estimate can never exceed the true remaining
        travel time.
        """
        def heuristic(node, goal):
            return cls.straight_line_distance(G, node, goal) / max_speed_mps
        return heuristic

    def straightLineTravelHeuristic(self, G: MultiDiGraph, current_node, goal_node, speed):
        return self.travelTime(speed, self.straightLineHeuristic(G, current_node=current_node, goal_node=goal_node))

    def travelTime(self, speed, length) -> int:
        return length / speed
            