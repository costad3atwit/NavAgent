from dataclasses import dataclass


@dataclass
class SearchMetrics:
    """
    Quantitative results of one search run. The algorithms fill in
    nodes_expanded/nodes_pruned; the runner fills in the rest.

    nodes_expanded: nodes whose successors were generated.
    nodes_pruned:   successors generated but not added to the frontier
                    (already visited / no g-score improvement); beam search
                    also counts nodes cut by the beam width each level.
    """
    nodes_expanded: int = 0
    nodes_pruned: int = 0
    compute_time_s: float = 0.0
    peak_memory_bytes: int = 0
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
            print("Utilizes highways: N/A (no route found)")
        elif self.uses_highways:
            print(f"Utilizes highways: Yes ({self.highway_edge_count} highway edges, "
                  f"{self.highway_distance_pct:.1f}% of route distance)")
        else:
            print("Utilizes highways: No")
