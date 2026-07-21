import networkx as nx
import osmnx as ox
import agents as a
import time

graphml_path = "data/weymouth.graphml"

G = ox.io.load_graphml(filepath=graphml_path)
Gp = ox.projection.project_graph(G)
fig, ax = ox.plot.plot_graph(G)