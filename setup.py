from pathlib import Path
import osmnx as ox

from util import Util

def saveGraph(location=(-71.21046,42.24448,-70.92035,42.37891), save_path="data/bostonbbox.graphml", network_type="drive", simplify=False, retain_all=False, truncate_by_edge=True):
    # Determine location variable type
    is_tuple = isinstance(location, tuple)
    is_string = isinstance(location, str)
    print("Verifying graph saving parameters:\n")
    print(f"Location type is {type(location)}\n")
    print(f"`is_tuple` evaluated to {is_tuple}\n")
    print(f"`is_string` evaluates to {is_string}\n")

    # Create branching condition based on the location variable type
    print("[Attempting graph retreival]")
    if is_tuple:
        G = ox.graph.graph_from_bbox(location, network_type=network_type, 
                             simplify=simplify, retain_all=retain_all, truncate_by_edge=truncate_by_edge)
    elif is_string: 
        G = ox.graph.graph_from_place(location, network_type=network_type, simplify=simplify)
    else:
        print("`location` must be a string or a tuple (LEFT, BOTTOM, LEFT, TOP)")
        return

    # Add edge speeds
    print("[Adding edge speeds...]")
    G = ox.routing.add_edge_speeds(G)
    G = ox.routing.add_edge_travel_times(G)

    # Tag highway edges so is_highway is baked into the saved graph
    print("[Tagging highway edges...]")
    Util.add_highway_flags(G)

    # Save the graph
    print("[Attempting to save graph...]")
    ox.io.save_graphml(G,filepath=save_path, gephi=False, encoding='utf-8')
    print("Graph successfully saved to disc.") 

if __name__ == "__main__":
    saveGraph()