#class for BFS agent
#          A* agent
#          Beam Search agent

class SearchAgents:

    def BFSagent():
        # TODO: Implement the BFSagent
        return None
    
    def AstarAgent():
        # TODO: Implement the A* agent
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
        return None
    