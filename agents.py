import itertools
import networkx as nx
import heapq
from util import Util

class SearchAgents:

    def BFSagent():
        # TODO: Implement the BFSagent
        return None
    
    def AstarAgent():
        def astar(G, start, goal, heuristic, weight_func):
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
                    return reconstruct_path(came_from, current)

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

        def reconstruct_path(came_from, current):
            path = []
            node = current
            while node in came_from:
                predecessor, edge_key = came_from[node]
                path.append((node, edge_key))
                node = predecessor
            path.append((node, None))   # start node: no incoming edge
            path.reverse()
            return path

        def speedAndDistanceHeuristic():
            #TODO: implement
            return None

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
        
    
    def BeamSearchAgent(G, start_node, goal_test, beam_width, max_depth):
        # TODO: Apply heuristic evaluation relevant to the Networkx Graph
        current_level = [start_node]
        depth = 0

        while current_level and depth < max_depth:
            all_successors = []

            # Generate all successors from current level
            for node in current_level:
                if node == goal_test:
                    return node # Success

                successors = nx.neighbors(G, node)
                for successor in successors:
                    value = Util.euclideanDistanceHeuristic(G=G, start_node=successor, goal_node=goal_test)
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
