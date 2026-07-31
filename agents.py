import itertools
import heapq
from util import Util
import networkx as nx


class AstarAgent:
    """
    Instantiate one AstarAgent and reuse it to run astar() over the same
    graph with different start/goal pairs for testing
    """

    def astar(self, G, start, goal, heuristic, weight_func):
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

    """
    def BeamSearchAgent(G, start_node, goal_test, beam_width):
        """
        
        """
        current_level = [start_node]
        depth = 0
        visited = {}

        while current_level:
            all_successors = []

            # Generate all successors from current level
            for node in current_level:
                if node == goal_test:
                    return node # Success

                successors = nx.neighbors(G, node)
                for successor in successors:
                    value = Util.straight_line_distance(G=G, start_node=successor, goal_node=goal_test)
                    all_successors.append(successor, value)

                if not all_successors:
                    return [] # Failure

                # Sort all successors by their scores (best first)
                sorted_successors = heapq.heapify(all_successors)

                # Keep only top beam_width candidates for next level
                current_level = sorted_successors[0 : beam_width]
                depth = depth + 1

        # Check final level for goal
        for node in current_level:
            if node == goal_test:
                return node # Success 

        return [] # Failure
