import itertools
import networkx as nx
import heapq

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


            return None

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
        
    
    def BeamSearchAgent():
        # TODO: Implement the BeamSearch agent
        """
        function BEAM_SEARCH(start_node, goal_test, beam_width, max_depth):
        current_level ← [start_node]
        depth ← 0

        while current_level is not empty AND depth < max_depth:
            all_successors ← []

            // Generate all successors from current level
            for each node in current_level:
                if goal_test(node):
                    return SUCCESS (node)

                successors ← get_successors(node)
                for each successor in successors:
                    successor.score ← evaluation_function(successor)
                    all_successors.append(successor)

            if all_successors is empty:
                return FAILURE

            // Sort all successors by their scores (best first)
            sorted_successors ← sort(all_successors, by=score, descending=True)

            // Keep only top beam_width candidates for next level
            current_level ← sorted_successors[0 : beam_width]
            depth ← depth + 1

        // Check final level for goal
        for each node in current_level:
            if goal_test(node):
                return SUCCESS (node)

        return FAILURE
        """
        fringe = util.Queue() # Potentially use heapqueue?
        fringe.push( (problem.getStartState(), []) )
        visited = set()

        while not fringe.isEmpty():
            state, actions = fringe.pop()

            if state in visited:
                continue
            visited.add(state)

            if problem.isGoalState(state):
                return actions

            for successor, action, stepCost in problem.getSuccessors(state):
                if successor not in visited:
                    fringe.push( (successor, actions + [action]) )

        return []
    