import osmnx as ox
from networkx.classes.multidigraph import MultiDiGraph

class Map:

    def load_map(path_to_map: str) -> tuple[MultiDiGraph, MultiDiGraph]:
        G = ox.io.load_graphml(filepath=path_to_map)
        # G = ox.routing.add_edge_speeds(G)
        # G = ox.routing.add_edge_travel_times(G)
        Gp = ox.projection.project_graph(G)

        return G, Gp

    # TODO: Constrain return value?
    def plot_map(graph: MultiDiGraph):
        fig, ax = ox.plot.plot_graph(graph)

        return fig, ax

    def plot_route(graph: MultiDiGraph, route):
        fig, ax = ox.plot.plot_graph_route(graph, route=route, route_color="y", route_linewidth=6, node_size=0)

        return fig, ax

    def plot_routes(graph: MultiDiGraph, routes, route_colors):
        fig, ax = ox.plot.plot_graph_routes(graph, routes=routes, route_colors=route_colors, route_linewidths=6, node_size=0)

        return fig, ax
