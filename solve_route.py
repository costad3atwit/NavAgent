import time
import tracemalloc

import osmnx as ox
from optparse import OptionParser
import sys

from agents import AstarAgent, BeamSearchAgent, BasicAgents
from map import Map
from util import Util

def runSearch():

    options = readCommand( sys.argv[1:] )

    print("Loading map and building projected graph...")
    graph, projected_graph = Map.load_map("data/bostonbbox.graphml")
    print(f"Graph loaded: {len(projected_graph.nodes)} nodes, {len(projected_graph.edges)} edges")


    start_goal_pairs = []
    # (lat, lon) test locations
    start_goal_pairs.append(((42.330397, -71.103089),(42.359206, -71.067703))) #hillside market -> beacon hill market

    start_goal_pairs.append(((42.251209, -71.005344),(42.366465, -71.054860))) #quincy center -> paul revere library



    for (start_latlon, goal_latlon) in start_goal_pairs:
        print("Snapping start and goal to nearest graph nodes...")
        start = ox.distance.nearest_nodes(graph, X=start_latlon[1], Y=start_latlon[0])
        goal = ox.distance.nearest_nodes(graph, X=goal_latlon[1], Y=goal_latlon[0])
        print(f"Start node: {start}, Goal node: {goal}")

        #was having issue with a path not existing because the graph is directed but cuts off with a hard boundary
        #meaning we can't solve for any route even if it exists IRL
        if not Util.canReachGoal(projected_graph, start, goal):
            raise SystemExit(f"No directed path exists between {start} and {goal} -- pick different coordinates")

        max_speed_kph = max(data.get("speed_kph", 0) for _, _, data in projected_graph.edges(data=True))
        max_speed_mps = max_speed_kph * 1000 / 3600
        print(f"Max edge speed in graph: {max_speed_kph:.1f} kph ({max_speed_mps:.2f} m/s)")

        heuristic = Util.speed_and_distance_heuristic(projected_graph, max_speed_mps)

        if options.agent == 'astar':
            agent = AstarAgent()
            def search():
                return agent.astar(
                    projected_graph,
                    start,
                    goal,
                    heuristic=heuristic,
                    weight_func=Util.travel_time_weight,
                )

        elif options.agent == 'beamsearch':
            agent = BeamSearchAgent()
            def search():
                return agent.beam_search(
                    G=projected_graph,
                    start=start,
                    goal=goal,
                    beam_width=options.beamwidth,
                    heuristic=heuristic,
                    weight_func=Util.travel_time_weight,
                )

        elif options.agent == 'dfs':
            agent = BasicAgents()
            def search():
                return agent.depth_first_search(
                    G=projected_graph,
                    start=start,
                    goal=goal,
                    weight_func=Util.travel_time_weight,
                )

        elif options.agent == 'bfs':
            agent = BasicAgents()
            def search():
                return agent.breadth_first_search(
                    G=projected_graph,
                    start=start,
                    goal=goal,
                    weight_func=Util.travel_time_weight,
                )

        else:
            print("Agent not specified correct")
            return

        # timed run
        print(f"Running {options.agent}...")
        search_start_time = time.perf_counter()
        path, metrics = search()
        metrics.compute_time_s = time.perf_counter() - search_start_time
        print(f"{options.agent} finished in {metrics.compute_time_s:.3f}s")

        # second run under tracemalloc for peak memory; traced separately
        # because tracing slows the search down and would skew the timing
        print("Tracing memory usage (re-running the search, may take a while)...")
        tracemalloc.start()
        search()
        metrics.peak_memory_bytes = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()

        if path is not None:
            (metrics.uses_highways,
             metrics.highway_edge_count,
             metrics.highway_distance_pct) = Util.highway_stats(projected_graph, path)

        metrics.report(options.agent)

        if path is None:
            print(f"No route found between {start} and {goal}")
        else:
            print(f"Fastest route found with {len(path)} nodes")
            fastest_route = [node for node, _ in path]

            print("Computing shortest (by distance) route for comparison...")
            shortest_route = ox.routing.shortest_path(G=projected_graph, orig=start, dest=goal, weight="length")

            print(f"Shortest route found with {len(shortest_route)} nodes, plotting both routes...")
            fig, ax = Map.plot_routes(
                projected_graph,
                routes=[fastest_route, shortest_route],
                route_colors=["y", "c"],
            )
            print("Done.")

def readCommand(argv):
    usageStr = """
    USAGE:      python solve_route.py [-a | --agent] [<astar | beamsearch | dfs | bfs>] 
    EXAMPLES:   (1) python solve_route.py
                    - run an A* search
                (2) python solve_route.py --agent beamsearch -b 7
                OR python solve_route.py -a beamsearch -beamwidth 7
                    - run a beam search with beamwidth 7
    """
    parser = OptionParser(usageStr)

    parser.add_option('-a', '--agent', dest='agent', type='str',
                      help='the agent to navigate the graph', metavar='AGENT', default='astar')
    
    parser.add_option('-b', '--beamwidth', dest='beamwidth', type='int',
                      help='beam width if using beam search', metavar='BEAMWIDTH', default='7')

    options, otherjunk = parser.parse_args(argv)
    if len(otherjunk) != 0:
        raise Exception('Command line input not understood: ' + str(otherjunk))
    args = dict()

    return options

if __name__ == '__main__':
    runSearch()