from pathlib import Path
import osmnx as ox

"""
Can change the parameters of ox.graph to specify a boundary box instead
"""

save_path = "data/bostonbbox.graphml" # Save path must end in .graphml extension
location = "Weymouth, Massachusetts, USA" # Location to save (Can use different functions to create different graph)

# G = ox.graph.graph_from_place(location, network_type="drive", simplify=False) # Graph from place 
G = ox.graph.graph_from_bbox((-71.21046,42.24448,-70.92035,42.37891), network_type="drive", 
                             simplify=False, retain_all=False, truncate_by_edge=True) # Boston BBox

ox.io.save_graphml(G, filepath=save_path, gephi=False, encoding='utf-8')
