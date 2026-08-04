from dataclasses import dataclass, field


@dataclass
class SearchMetrics:
    """
    Quantitative results of one search run. The algorithms fill in
    nodes_expanded/nodes_pruned; the runner fills in the rest.

    nodes_expanded: nodes whose successors were generated.
    nodes_pruned:   successors generated but not added to the frontier
                    (already visited / no g-score improvement); beam search
                    also counts nodes cut by the beam width each level.
    explored_edges: every (u, v, key) edge expanded during the search,
                    for plotting; empty when recording is disabled.
    """
    explored_edges: set = field(default_factory=set, repr=False)
    nodes_expanded: int = 0
    nodes_pruned: int = 0
    compute_time_s: float = 0.0
    peak_memory_bytes: int = 0
    route_travel_time_s: float = 0.0
    route_distance_m: float = 0.0
    shortest_route_travel_time_s: float = 0.0
    shortest_route_distance_m: float = 0.0
    uses_highways: bool = None
    highway_edge_count: int = 0
    highway_distance_pct: float = 0.0

    def report(self, agent_name):
        print(f"\n=== Metrics: {agent_name} ===")
        print(f"Compute time:      {self.compute_time_s:.3f} s")
        print(f"Peak memory:       {self.peak_memory_bytes / (1024 * 1024):.1f} MiB")
        print(f"Nodes expanded:    {self.nodes_expanded:,}")
        print(f"Nodes pruned:      {self.nodes_pruned:,}")
        if self.uses_highways is None:
            print("Route travel time: N/A (no route found)")
            print("Route distance:    N/A (no route found)")
            print("Utilizes highways: N/A (no route found)")
            return
        print(f"Route travel time: {self.route_travel_time_s:,.1f} s "
              f"({self.route_travel_time_s / 60:.1f} min; "
              f"shortest route: {self.shortest_route_travel_time_s:,.1f} s)")
        print(f"Route distance:    {self.route_distance_m:,.0f} m "
              f"(shortest route: {self.shortest_route_distance_m:,.0f} m)")
        if self.uses_highways:
            print(f"Utilizes highways: Yes ({self.highway_edge_count} highway edges, "
                  f"{self.highway_distance_pct:.1f}% of route distance)")
        else:
            print("Utilizes highways: No")
