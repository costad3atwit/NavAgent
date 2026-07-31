from math import sqrt

import networkx as nx
import osmnx as ox
from networkx.classes.multidigraph import MultiDiGraph


class Util:

    def canReachGoal(self, G: MultiDiGraph, start_node, goal_node):
        return goal_node in nx.descendants(G, start_node)

    def straightLineHeuristic(self, G: MultiDiGraph, current_node=None, goal_node=None):

        if self.canReachGoal(G, current_node, goal_node):
            return 0

        if current_node and goal_node:
            x1 = current_node.get_node_attributes(G, 'x')
            y1 = current_node.get_node_attributes(G, 'y')

            x2 = goal_node.get_node_attributes(G, 'x')
            y2 = goal_node.get_node_attributes(G, 'y')

            return sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
        else:
            return 0

    def euclideanDistanceHeuristic(self, G: MultiDiGraph, current_node=None, goal_node=None):
        if self.canReachGoal:
            return 0
        route = ox.routing.shortest_path(G=G, orig=current_node, dest=goal_node, weight="length")
        total_length = 0

        i = 1
        while i < len(route):
            length = G.get_edge_data(route[i - 1], route[i], 'length')
            total_length += length
        return total_length

    def straightLineTravelHeuristic(self, G: MultiDiGraph, current_node, goal_node, speed):
        return self.travelTime(speed, self.straightLineHeuristic(G, current_node=current_node, goal_node=goal_node))

    def euclideanDistanceTravelHeuristic(self, G: MultiDiGraph, current_node, goal_node, speed):
        return self.travelTime(speed, self.euclideanDistanceHeuristic(G, current_node=current_node, goal_node=goal_node))

    def travelTime(self, speed, length) -> int:
        return length / speed
            