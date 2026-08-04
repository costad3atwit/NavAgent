import itertools
import heapq
from collections import deque

from metrics import SearchMetrics
from util import Util

import networkx as nx
from networkx import MultiDiGraph


class AstarAgent:
    """
    Instantiate one AstarAgent and reuse it to run astar() over the same
    graph with different start/goal pairs for testing
    """

    def astar(self, G: MultiDiGraph, start, goal, heuristic, weight_func, record_explored=True):
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

        Returns (path, SearchMetrics); path is None if no route exists.
        """
        from fileIO import createPath
        path = createPath(folder_name="astar_output", output_name="astar")
        output_file = open(path, "w")

        metrics = SearchMetrics()
        counter = itertools.count()
        open_set = []
        heapq.heappush(open_set, (heuristic(start, goal), next(counter), start)) # push start to the queue using heurisitc for priority
        came_from = {}              # for reconstructing the path afterwards
        g_score = { start: 0 }      # actual cost from start up to here
        visited = set()             # nodes we've already expanded/finalized

        while open_set:
            output_file.write(f"\nOpen set: {[x[2] for x in open_set]}")
            _, _, current = heapq.heappop(open_set) #take off the highest priority node (lowest f score) item

            if current == goal:
                return Util._reconstruct_path(came_from, current), metrics

            if current in visited:
                continue                     # skip already visited nodes
            visited.add(current)
            metrics.nodes_expanded += 1
            output_file.write(f"\nExpanding node: {current}")

            for neighbor in G.successors(current):     # outgoing edges only (it's a DiGraph)

                # there may be several parallel edges to this same neighbor
                # since it's a MultiDiGraph so we pick the cheapest one
                best_edge_key, best_edge_cost = None, float('inf')
                for key, edge_data in G[current][neighbor].items():
                    cost = weight_func(current, neighbor, edge_data)
                    if cost < best_edge_cost:
                        best_edge_cost = cost
                        best_edge_key = key

                if record_explored:
                    metrics.explored_edges.add((current, neighbor, best_edge_key))

                tentative_g = g_score[current] + best_edge_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = (current, best_edge_key)
                    priority = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, next(counter), neighbor))
                else:
                    metrics.nodes_pruned += 1
        output_file.close()
        return None, metrics   # no path exists

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
                    weight_func = Util.travel_time_weight,
                    record_explored = True):
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

        Returns (path, SearchMetrics); on failure the path covers only what
        was reached before the beam emptied.
        """
        from fileIO import createPath
        path = createPath(folder_name="beam_search_output", output_name="beam_search")
        output_file = open(path, "w")

        output_file.write("Initializing BEAM SEARCH...")
        metrics = SearchMetrics()
        current_level = [(heuristic(start, goal), start)]
        output_file.write(f"Current Level Nodes:\n{current_level}")
        came_from = {}
        g_score = { start: 0 }

        while current_level:
            output_file.write(f"\nCurrent Level Nodes:\n{[x[1] for x in current_level]}")
            all_successors = []

            # Generate all successors from current level
            for _, current_node in current_level:
                if current_node == goal:
                    # print("[SUCCESS, GOAL REACHED]")
                    output_file.write("\n[SUCCESS, GOAL REACHED]")
                    output_file.close()
                    return Util._reconstruct_path(came_from=came_from, current=current_node), metrics # Success

                metrics.nodes_expanded += 1
                for successor in G.successors(current_node):
                    output_file.write(f"\nExpanding node: {successor}")
                    best_edge_key, best_edge_cost = None, float('inf')
                    for key, edge_data in G[current_node][successor].items():
                        cost = weight_func(current_node, successor, edge_data)
                        if cost < best_edge_cost:
                            best_edge_cost = cost
                            best_edge_key = key

                    if record_explored:
                        metrics.explored_edges.add((current_node, successor, best_edge_key))

                    current_g = g_score[current_node] + best_edge_cost

                    if successor not in g_score or current_g < g_score[successor]:
                        g_score[successor] = current_g
                        came_from[successor] = (current_node, best_edge_key)
                        value = current_g + heuristic(successor, goal)
                        heapq.heappush(all_successors, (value, successor))
                    else:
                        metrics.nodes_pruned += 1

            if not all_successors:
                # print("[FAILURE, NO SUCCESSORS]")
                output_file.write("\n[FAILURE, NO SUCCESSORS]")
                output_file.close()
                return Util._reconstruct_path(came_from=came_from, current=current_node), metrics # Failure

            # Keep only top beam_width candidates for next level;
            # everything cut by the beam width counts as pruned
            output_file.write(f"\nAll successors:\n{[x[1] for x in all_successors]}")
            metrics.nodes_pruned += max(0, len(all_successors) - beam_width)
            current_level = heapq.nsmallest(beam_width, all_successors)
            output_file.write(f"\nPruned list:\n{[x[1] for x in current_level]}\n")
        output_file.close()
        return None, metrics # Failure


class BasicAgents:
    def depth_first_search(self,
                           G: MultiDiGraph, 
                           start: int, 
                           goal: int,
                           weight_func=Util.travel_time_weight,
                           record_explored=True):
        from fileIO import createPath
        path = createPath(folder_name="dfs_output", output_name="dfs")
        output_file = open(path, "w")
        
        metrics = SearchMetrics()
        frontier = []
        frontier.append( start )
        visited = set()
        came_from = {}

        while frontier:
            current_node = frontier.pop()

            if current_node in visited:
                continue

            visited.add(current_node)
            output_file.write(f"\nExpanding node: {current_node}")
            if current_node == goal:
                return Util._reconstruct_path(came_from=came_from, current=current_node), metrics

            metrics.nodes_expanded += 1

            for successor in G.successors(current_node):

                best_edge_key, best_edge_cost = None, float('inf')
                for key, edge_data in G[current_node][successor].items():
                    cost = weight_func(current_node, successor, edge_data)
                    if cost < best_edge_cost:
                        best_edge_cost = cost
                        best_edge_key = key

                if record_explored:
                    metrics.explored_edges.add((current_node, successor, best_edge_key))

                if successor not in visited:
                    came_from[successor] = (current_node, best_edge_key)
                    frontier.append( successor )
                else:
                    metrics.nodes_pruned += 1
            output_file.write(f"\nFrontier: {frontier}\n")

        output_file.close()
        return None, metrics

    def breadth_first_search(self,
                             G: MultiDiGraph,
                             start: int,
                             goal: int,
                             weight_func = Util.travel_time_weight,
                             record_explored = True):
        from fileIO import createPath
        path = createPath(folder_name="bfs_output", output_name="bfs")
        output_file = open(path, "w")
        
        metrics = SearchMetrics()
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
                return Util._reconstruct_path(came_from=came_from, current=current_node), metrics

            metrics.nodes_expanded += 1
            output_file.write(f"\nExpanding node: {current_node}")
            for successor in G.successors(current_node):

                best_edge_key, best_edge_cost = None, float('inf')
                for key, edge_data in G[current_node][successor].items():
                    cost = weight_func(current_node, successor, edge_data)
                    if cost < best_edge_cost:
                        best_edge_cost = cost
                        best_edge_key = key

                if record_explored:
                    metrics.explored_edges.add((current_node, successor, best_edge_key))

                if successor not in visited:
                    came_from[successor] = (current_node, best_edge_key)
                    frontier.append( successor )
                else:
                    metrics.nodes_pruned += 1
            output_file.write(f"\nFrontier: {frontier}\n")
        output_file.close()
        return None, metrics