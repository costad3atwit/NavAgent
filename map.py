import networkx as nx
from networkx.classes.multidigraph import MultiDiGraph
import osmnx as ox
import agents as a
import time

class Map:

    def load_map(path_to_map: str) -> MultiDiGraph:
        G = ox.io.load_graphml(filepath=path_to_map)
        Gp = ox.projection.project_graph(G)

        return Gp

    # TODO: Constrain return value?
    def plot_map(graph: MultiDiGraph):
        fig, ax = ox.plot.plot_graph(projected_graph)

        return fig, ax


projected_graph = Map.load_map("data/weymouth.graphml")
fig, ax = Map.plot_map(projected_graph)