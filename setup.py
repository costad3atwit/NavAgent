from pathlib import Path
import osmnx as ox

def saveGraph(location=(-71.21046,42.24448,-70.92035,42.37891), network_type="drive", simplify=False, retain_all=False, truncate_by_edge=True, save_path="data/bostonbbox.graphml"):
    # TODO: Determine location variable type
    is_tuple = isinstance(location, tuple)
    is_string = isinstance(location, str)
    print(f"Location type is {type(location)}\n")
    print(f"`is_tuple` evaluated to {is_tuple}\n")
    print(f"`is_string` evaluates to {is_string}\n")

    # TODO: Create branching condition based on the location variable type
    if is_tuple:
        G = ox.graph.graph_from_bbox(location, network_type="drive", 
                             simplify=False, retain_all=False, truncate_by_edge=True)
    elif is_string: 
        G = ox.graph.graph_from_place(location, network_type="drive", simplify=False)
    else:
        print("`location` must be a string or a tuple (LEFT, BOTTOM, LEFT, TOP)")
        return

    # TODO: Save the graph
    ox.io.save_graphml(G,filepath=save_path, gephi=False, encoding='utf-8')

# saveGraph(location="Weymouth, Massachusetts, USA", save_path="data/weymouth.graphml")