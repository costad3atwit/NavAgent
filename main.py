import networkx as nx
import osmnx as ox
import agents as a
import time


# download/model a street network for some city then visualize it

# Start the timer
start_graph_from_place = time.perf_counter()
G = ox.graph.graph_from_place("Boston, Massachusetts, USA", network_type="drive")
Gp = ox.projection.project_graph(G)
end_graph_from_place = time.perf_counter()
elapsed = end_graph_from_place - start_graph_from_place
#fig, ax = ox.plot.plot_graph(G)

# find the shortest path (by distance) between these nodes then plot it
points = ox.utils_geo.sample_points(ox.convert.to_undirected(Gp), n=100)
X = points.x.values
Y = points.y.values
X0 = X.min()
Y0 = Y.min()

X1 = X.max()
Y1 = Y.max()

orig = ox.distance.nearest_nodes(Gp, X0, Y0)
dest = ox.distance.nearest_nodes(Gp, X1, Y1)
route = ox.routing.shortest_path(G, orig, dest, weight="length")
fig, ax = ox.plot.plot_graph_route(G, route, route_color="y", route_linewidth=6, node_size=0)