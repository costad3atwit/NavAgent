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
    match options.route:
        case 'hillside-beacon':
            start_goal_pairs.append(((42.330397, -71.103089),(42.359206, -71.067703))) #Hillside Market -> Beacon Hill Market
        case 'quincy-library':
            start_goal_pairs.append(((42.251209, -71.005344),(42.366465, -71.054860))) # Quincy Center -> Paul Revere Library
        case 'bc-ahern':
            start_goal_pairs.append(((42.334004,-71.166438),(42.368669,-71.085486))) # Boston College -> Ahern Field (Cambridge)
        case 'logan-franklin':
            start_goal_pairs.append(((42.368885,-71.018870),(42.302347,-71.089611))) # Terminal E (Logan) -> Franklin Park zoo
        case 'all':
            start_goal_pairs.append(((42.330397, -71.103089),(42.359206, -71.067703))) #Hillside Market -> Beacon Hill Market
            start_goal_pairs.append(((42.251209, -71.005344),(42.366465, -71.054860))) # Quincy Center -> Paul Revere Library
            start_goal_pairs.append(((42.334004,-71.166438),(42.368669,-71.085486))) # Boston College -> Ahern Field (Cambridge)
            start_goal_pairs.append(((42.368885,-71.018870),(42.302347,-71.089611))) # Terminal E (Logan) -> Franklin Park zoo

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

        match options.agent:
            case'astar':
                agent = AstarAgent()
                def search(record_explored=True):
                    return agent.astar(
                        projected_graph,
                        start,
                        goal,
                        heuristic=heuristic,
                        weight_func=Util.travel_time_weight,
                        record_explored=record_explored,
                    )

            case 'beamsearch':
                agent = BeamSearchAgent()
                def search(record_explored=True):
                    return agent.beam_search(
                        G=projected_graph,
                        start=start,
                        goal=goal,
                        beam_width=options.beamwidth,
                        heuristic=heuristic,
                        weight_func=Util.travel_time_weight,
                        record_explored=record_explored,
                    )

            case 'dfs':
                agent = BasicAgents()
                def search(record_explored=True):
                    return agent.depth_first_search(
                        G=projected_graph,
                        start=start,
                        goal=goal,
                        weight_func=Util.travel_time_weight,
                        record_explored=record_explored,
                    )

            case 'bfs':
                agent = BasicAgents()
                def search(record_explored=True):
                    return agent.breadth_first_search(
                        G=projected_graph,
                        start=start,
                        goal=goal,
                        weight_func=Util.travel_time_weight,
                        record_explored=record_explored,
                    )

            case _:
                print("Agent not specified correctly\nUSAGE: python solve_route.py [-a | --agent] [<astar | beamsearch | dfs | bfs>]")
                return

        # timed run
        print(f"Running {options.agent}...")
        search_start_time = time.perf_counter()
        path, metrics = search()
        metrics.compute_time_s = time.perf_counter() - search_start_time
        print(f"{options.agent} finished in {metrics.compute_time_s:.3f}s")

        # second run under tracemalloc for peak memory; traced separately
        # because tracing slows the search down and would skew the timing,
        # and without explored-edge recording so the extra bookkeeping
        # doesn't inflate the peak
        print("Tracing memory usage (re-running the search, may take a while)...")
        tracemalloc.start()
        search(record_explored=False)
        metrics.peak_memory_bytes = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()

        if path is not None:
            (metrics.uses_highways,
             metrics.highway_edge_count,
             metrics.highway_distance_pct,
             metrics.route_distance_m) = Util.highway_stats(projected_graph, path)
            metrics.route_travel_time_s = Util.path_travel_time(projected_graph, path)

            print("Computing shortest (by distance) route for comparison...")
            shortest_route = ox.routing.shortest_path(G=projected_graph, orig=start, dest=goal, weight="length")
            (metrics.shortest_route_distance_m,
             metrics.shortest_route_travel_time_s) = Util.route_stats(projected_graph, shortest_route)

        metrics.report(options.agent)

        if path is None:
            print(f"No route found between {start} and {goal}")
        else:
            print(f"Fastest route found with {len(path)} nodes")
            fastest_route = [node for node, _ in path]

            print(f"Shortest route found with {len(shortest_route)} nodes, plotting both routes...")
            fig, ax = Map.plot_routes(
                projected_graph,
                routes=[fastest_route, shortest_route],
                route_colors=["y", "c"],
                explored_edges=metrics.explored_edges,
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
                      help='the agent to navigate the graph [-a | --agent] [<astar | beamsearch | dfs | bfs>]', metavar='AGENT', default='astar')
    
    parser.add_option('-b', '--beamwidth', dest='beamwidth', type='int',
                      help='beam width if using beam search', metavar='BEAMWIDTH', default='7')
    
    parser.add_option('-r', '--route', dest="route", type='str',
                      help='which goal destination pair to solve', metavar='ROUTE', default='all')

    options, otherjunk = parser.parse_args(argv)
    if len(otherjunk) != 0:
        raise Exception('Command line input not understood: ' + str(otherjunk))

    return options

if __name__ == '__main__':
    runSearch()