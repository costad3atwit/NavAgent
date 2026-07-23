import networkx as nx
import osmnx as ox
import agents as a
import time

"""
Change graphml file to switch which files to load
"""
graphml_path = "data/weymouth.graphml"

G = ox.io.load_graphml(filepath=graphml_path)
Gp = ox.projection.project_graph(G)

print(type(G))
print()
print(type(Gp))