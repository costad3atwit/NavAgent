import osmnx as ox
from networkx.classes.multidigraph import MultiDiGraph

class Map:

    def load_map(path_to_map: str) -> tuple[MultiDiGraph, MultiDiGraph]:
        # graphml stores every attribute as a string, so the is_highway flag
        # baked in by setup.py must be converted back to a real boolean
        G = ox.io.load_graphml(
            filepath=path_to_map,
            edge_dtypes={"is_highway": lambda value: value == "True"},
        )
        Gp = ox.projection.project_graph(G)

        return G, Gp

    # TODO: Constrain return value?
    def plot_map(graph: MultiDiGraph):
        fig, ax = ox.plot.plot_graph(graph)

        return fig, ax

    def plot_route(graph: MultiDiGraph, route):
        fig, ax = ox.plot.plot_graph_route(graph, route=route, route_color="y", route_linewidth=6, node_size=0)

        return fig, ax

    def plot_routes(graph: MultiDiGraph, routes, route_colors, explored_edges=None):
        # highlight edges touched by the search; extra kwargs are forwarded
        # by osmnx down to plot_graph, which takes a per-edge color list
        # ordered like graph.edges(keys=True)
        kwargs = {}
        if explored_edges:
            kwargs["edge_color"] = [
                "#d95f0e" if edge in explored_edges else "#333333"
                for edge in graph.edges(keys=True)
            ]

        fig, ax = ox.plot.plot_graph_routes(graph, routes=routes, route_colors=route_colors, route_linewidths=6, node_size=0, **kwargs)

        return fig, ax
