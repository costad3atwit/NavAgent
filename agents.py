import itertools
import heapq
from collections import deque

from util import Util

import networkx as nx
from networkx import MultiDiGraph


class AstarAgent:
    """
    Instantiate one AstarAgent and reuse it to run astar() over the same
    graph with different start/goal pairs for testing
    """

    def astar(self, G: MultiDiGraph, start, goal, heuristic, weight_func):
        """
        heuristic(node, goal) -> estimated cost from `node` to `goal`.
        Must be admissible (never overestimate the true remaining cost) or the
        path found isn't guaranteed optimal. Whatever units it returns must
        match weight_func's units (e.g. don't mix a distance heuristic with
        a time-based weight_func).

        weight_func(u, v, edge_data) -> cost of a single edge from u to v,
        where edge_data is one parallel edge's attribute dict from a
        MultiDiGraph (G[u][v][key]). Called once per parallel edge so the
        cheapest of several u->v edges can be picked.

        g_score[n] = best known cost from start to n
        f_score[n] = g_score[n] + heuristic(n, goal)  -> priority in the queue
        """

        counter = itertools.count()
        open_set = []
        heapq.heappush(open_set, (heuristic(start, goal), next(counter), start)) # push start to the queue using heurisitc for priority
        came_from = {}              # for reconstructing the path afterwards
        g_score = { start: 0 }      # actual cost from start up to here
        visited = set()             # nodes we've already expanded/finalized

        while open_set:
            _, _, current = heapq.heappop(open_set) #take off the highest priority node (lowest f score) item

            if current == goal:
                return Util._reconstruct_path(came_from, current)

            if current in visited:
                continue                     # skip already visited nodes
            visited.add(current)

            for neighbor in G.successors(current):     # outgoing edges only (it's a DiGraph)

                # there may be several parallel edges to this same neighbor
                # since it's a MultiDiGraph so we pick the cheapest one
                best_edge_key, best_edge_cost = None, float('inf')
                for key, edge_data in G[current][neighbor].items():
                    cost = weight_func(current, neighbor, edge_data)
                    if cost < best_edge_cost:
                        best_edge_cost = cost
                        best_edge_key = key

                tentative_g = g_score[current] + best_edge_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = (current, best_edge_key)
                    priority = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, next(counter), neighbor))

        return None   # no path exists

class BeamSearchAgent:
    """
    Beam search class containing the algorithm for beam search.
    Reusable over different graphs, starts/goals, etc.
    """
    def beam_search(self, G: MultiDiGraph, 
                    start: int, 
                    goal: int, 
                    beam_width: int, 
                    heuristic = Util.speed_and_distance_heuristic, 
                    weight_func = Util.travel_time_weight):
        """
        Like A* search, beam search also uses a heuristic evaluation. Unlike
        A*, beam search uses this evaluation to prune nodes of less priority
        and keep an arbitrary amount to explore (the beam width). 

        heuristic(node, goal) -> estimated cost from `node` to `goal`.
        Must be admissible (never overestimate the true remaining cost) or the
        path found isn't guaranteed optimal. Whatever units it returns must
        match weight_func's units (e.g. don't mix a distance heuristic with
        a time-based weight_func).

        weight_func(u, v, edge_data) -> cost of a single edge from u to v,
        where edge_data is one parallel edge's attribute dict from a
        MultiDiGraph (G[u][v][key]). Called once per parallel edge so the
        cheapest of several u->v edges can be picked.

        g_score[n] = best known cost from start to n
        f_score[n] = g_score[n] + heuristic(n, goal)  -> priority in the queue
        """
        
        print("Initializing BEAM SEARCH...")
        current_level = [(heuristic(start, goal), start)]
        print(f"Current_level contains nodes: {current_level}")
        came_from = {}
        g_score = { start: 0 }

        while current_level:
            all_successors = []

            # Generate all successors from current level
            for _, current_node in current_level:
                if current_node == goal:
                    print("[SUCCESS, GOAL REACHED]")
                    return Util._reconstruct_path(came_from=came_from, current=current_node) # Success

                for successor in G.successors(current_node):
                    best_edge_key, best_edge_cost = None, float('inf')
                    for key, edge_data in G[current_node][successor].items():
                        cost = weight_func(current_node, successor, edge_data)
                        if cost < best_edge_cost:
                            best_edge_cost = cost
                            best_edge_key = key

                    current_g = g_score[current_node] + best_edge_cost

                    if successor not in g_score or current_g < g_score[successor]:
                        g_score[successor] = current_g
                        came_from[successor] = (current_node, best_edge_key)
                        value = current_g + heuristic(successor, goal)
                        heapq.heappush(all_successors, (value, successor))
                    else: continue

            if not all_successors:
                print("[FAILURE, NO SUCCESSORS]")
                return Util._reconstruct_path(came_from=came_from, current=current_node) # Failure

            # Keep only top beam_width candidates for next level
            current_level = heapq.nsmallest(beam_width, all_successors)

        return None # Failure


class BasicAgents:
    def depth_first_search(self,
                           G: MultiDiGraph, 
                           start: int, 
                           goal: int,
                           weight_func=Util.travel_time_weight):
        frontier = []
        frontier.append( start )
        visited = set()
        came_from = {}

        while frontier:
            current_node = frontier.pop()

            if current_node in visited:
                continue

            visited.add(current_node)

            if current_node == goal:
                return Util._reconstruct_path(came_from=came_from, current=current_node)

            for successor in G.successors(current_node):

                best_edge_key, best_edge_cost = None, float('inf')
                for key, edge_data in G[current_node][successor].items():
                    cost = weight_func(current_node, successor, edge_data)
                    if cost < best_edge_cost:
                        best_edge_cost = cost
                        best_edge_key = key

                if successor not in visited:
                    came_from[successor] = (current_node, best_edge_key)
                    frontier.append( successor )

        return None

    def breadth_first_search(self,
                             G: MultiDiGraph,
                             start: int,
                             goal: int,
                             weight_func = Util.travel_time_weight):
        frontier = deque()
        frontier.append( start )
        visited = set()
        came_from = {}

        while frontier:
            current_node = frontier.popleft()

            if current_node in visited:
                continue

            visited.add(current_node)

            if current_node == goal:
                return Util._reconstruct_path(came_from=came_from, current=current_node)

            for successor in G.successors(current_node):

                best_edge_key, best_edge_cost = None, float('inf')
                for key, edge_data in G[current_node][successor].items():
                    cost = weight_func(current_node, successor, edge_data)
                    if cost < best_edge_cost:
                        best_edge_cost = cost
                        best_edge_key = key

                if successor not in visited:
                    came_from[successor] = (current_node, best_edge_key)
                    frontier.append( successor )

        return None