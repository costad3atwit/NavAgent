import time

import networkx as nx
import osmnx as ox

from agents import AstarAgent, SearchAgents
from map import Map
from util import Util


print("Loading map and building projected graph...")
graph, projected_graph = Map.load_map("data/bostonbbox.graphml")
print(f"Graph loaded: {len(projected_graph.nodes)} nodes, {len(projected_graph.edges)} edges")

# (lat, lon) test locations
start_latlon = (42.330397, -71.103089)
goal_latlon = (42.359206, -71.067703)

print("Snapping test coordinates to nearest graph nodes...")
start = ox.distance.nearest_nodes(graph, X=start_latlon[1], Y=start_latlon[0])
goal = ox.distance.nearest_nodes(graph, X=goal_latlon[1], Y=goal_latlon[0])
print(f"Start node: {start}, Goal node: {goal}")

#was having issue with a path not existing because the graph is directed but cuts off with a hard boundary
#meaning we can't solve for any route even if it exists IRL
if not nx.has_path(projected_graph, start, goal):
    raise SystemExit(f"No directed path exists between {start} and {goal} -- pick different coordinates")

max_speed_kph = max(data.get("speed_kph", 0) for _, _, data in projected_graph.edges(data=True))
max_speed_mps = max_speed_kph * 1000 / 3600
print(f"Max edge speed in graph: {max_speed_kph:.1f} kph ({max_speed_mps:.2f} m/s)")

astar_agent = AstarAgent()
heuristic = Util.speed_and_distance_heuristic(projected_graph, max_speed_mps)

print("Running AstarAgent.astar()...")
astar_start_time = time.perf_counter()
path = astar_agent.astar(
    projected_graph,
    start,
    goal,
    heuristic=heuristic,
    weight_func=SearchAgents.travel_time_weight,
)
astar_elapsed = time.perf_counter() - astar_start_time
print(f"astar() finished in {astar_elapsed:.3f}s")

if path is None:
    print(f"No route found between {start} and {goal}")
else:
    print(f"Fastest route found with {len(path)} nodes")
    fastest_route = [node for node, _ in path]

    print("Computing shortest (by distance) route for comparison...")
    shortest_route = ox.routing.shortest_path(G=projected_graph, orig=start, dest=goal, weight="length")

    if shortest_route is None:
        print(f"No shortest-distance route found between {start} and {goal}")
        fig, ax = Map.plot_route(projected_graph, fastest_route)
    else:
        print(f"Shortest route found with {len(shortest_route)} nodes, plotting both routes...")
        fig, ax = Map.plot_routes(
            projected_graph,
            routes=[fastest_route, shortest_route],
            route_colors=["y", "c"],
        )
    print("Done.")