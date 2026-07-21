import networkx as nx
import osmnx as ox
import agents as a
import time


# download/model a street network for some city then visualize it

# Start the timer
print('Starting Graph Retrieval')
start_graph_from_place = time.perf_counter()
# G = ox.graph.graph_from_place("Boston, Massachusetts, USA", network_type="drive")
graphml_path = "data/bostonbbox.graphml"
G = ox.io.load_graphml(filepath=graphml_path)
Gp = ox.projection.project_graph(G)
end_graph_from_place = time.perf_counter()
elapsed_download = end_graph_from_place - start_graph_from_place
print(f"Retreiving The Graph Took: {elapsed_download:.6f}")
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

print(f"Origin Node: {orig}\nDestination Node: {dest}\n")

print('Starting shortest path Computation')
path_calc_start=time.perf_counter()
route = ox.routing.shortest_path(G, orig=orig, dest=dest, weight="length") # Was not computing with load_graphml, had to use keyword arguments over positional arguments
print(f"Type of Route: {type(route)}\nRoute: {route}")
path_calc_end=time.perf_counter()
path_calc_time = path_calc_end - path_calc_start

print(f'Finding the shortest path took: {path_calc_time:.6f}')

"""
#FOR PROSPERO UNCOMMENT TO WRITE A FILE CONTAINING THE ROUTE
with open('output.txt', 'w') as file:
    file.write(','.join(route))

"""

#FOR PROSPERO, COMMENT
fig, ax = ox.plot.plot_graph_route(G, route, route_color="y", route_linewidth=6, node_size=0)