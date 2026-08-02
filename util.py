from math import sqrt

import networkx as nx
import osmnx as ox
from networkx.classes.multidigraph import MultiDiGraph


class Util:
    # OSM road classes we consider "highways" for the is_highway edge flag
    HIGHWAY_CLASSES = {"motorway", "motorway_link"}

    @staticmethod
    def canReachGoal(G: MultiDiGraph, start_node, goal_node):
        return goal_node in nx.descendants(G, start_node)

    @staticmethod
    def add_highway_flags(G: MultiDiGraph):
        # An edge's OSM 'highway' value is a single road class or a list of
        # classes (when OSMnx merged segments with different tags).
        for _, _, edge_data in G.edges(data=True):
            highway = edge_data.get("highway", "")
            classes = highway if isinstance(highway, list) else [highway]
            edge_data["is_highway"] = any(c in Util.HIGHWAY_CLASSES for c in classes)

    @staticmethod
    def highway_stats(G: MultiDiGraph, path):
        """
        path is [(node, edge_key), ...] as returned by the agents: entry i's
        edge_key identifies the edge from path[i-1]'s node into entry i's node.
        Returns (uses_highways, highway_edge_count, pct_of_route_distance,
        route_distance_m). Requires add_highway_flags() to have been run on G.
        """
        highway_count = 0
        highway_length = 0.0
        total_length = 0.0
        for (u, _), (v, key) in zip(path, path[1:]):
            edge_data = G[u][v][key]
            length = edge_data.get("length", 0.0)
            total_length += length
            if edge_data.get("is_highway"):
                highway_count += 1
                highway_length += length
        pct = (highway_length / total_length * 100) if total_length else 0.0
        return highway_count > 0, highway_count, pct, total_length

    @staticmethod
    def path_travel_time(G: MultiDiGraph, path):
        # Total travel time in seconds of a path of (node, edge_key) pairs as
        # returned by the agents -- the objective value of the found route.
        return sum(
            G[u][v][key]["travel_time"]
            for (u, _), (v, key) in zip(path, path[1:])
        )

    @staticmethod
    def route_stats(G: MultiDiGraph, route):
        # (distance_m, travel_time_s) of a route given as a plain list of node
        # ids (e.g. from ox.routing.shortest_path), taking the shortest
        # parallel edge between each consecutive pair -- the same edge a
        # length-weighted shortest path would use.
        total_length = 0.0
        total_travel_time = 0.0
        for u, v in zip(route, route[1:]):
            edge_data = min(G[u][v].values(), key=lambda d: d.get("length", 0.0))
            total_length += edge_data.get("length", 0.0)
            total_travel_time += edge_data.get("travel_time", 0.0)
        return total_length, total_travel_time

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
    def travel_time_weight(u, v, edge_data):
        # weight_func for fastest-route search. Requires the graph to have been
        # run through ox.routing.add_edge_speeds() and add_edge_travel_times()
        # beforehand so edge_data['travel_time'] exists.
        return edge_data['travel_time']

    @staticmethod
    def straight_line_distance(G, u, v):
        # Euclidean distance between two nodes' projected coordinates, in
        # the graph's projected units (meters after ox.projection.project_graph).
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def distance_weight(u, v, edge_data):
        # weight_func for shortest-distance search: cost of an edge is just its length.
        return edge_data['length']
    
    @staticmethod
    def travel_time_weight(u, v, edge_data):
        # weight_func for fastest-route search. Requires the graph to have been
        # run through ox.routing.add_edge_speeds() and add_edge_travel_times()
        # beforehand so edge_data['travel_time'] exists.
        return edge_data['travel_time']

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
            