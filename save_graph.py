from pathlib import Path
import osmnx as ox

"""
Can change the parameters of ox.graph to specify a boundary box instead
"""

save_path = "data/weymouth.graphml"
location = "Weymouth, Massachusetts, USA"

G = ox.graph.graph_from_place(location , network_type="drive", simplify=False)
ox.io.save_graphml(G, filepath=save_path, gephi=False, encoding='utf-8')
