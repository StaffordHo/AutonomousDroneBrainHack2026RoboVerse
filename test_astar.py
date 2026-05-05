import numpy as np
from GlobalMapper import GlobalMapper
from AStarPlanner import AStarPlanner

def test_astar_integration():
    K = np.array([[433.0, 0.0, 320.0], [0.0, 433.0, 240.0], [0.0, 0.0, 1.0]])
    mapper = GlobalMapper(K)
    
    # Create a vertical wall at North=5.0m, from East=-5.0 to East=5.0
    wall_points = []
    for e in np.linspace(-5, 5, 20):
        wall_points.append([5.0, e])
    mapper.global_points = np.array(wall_points)
    
    print(f"Mapper has {len(mapper.global_points)} points.")
    
    grid, origin = mapper.get_occupancy_grid(resolution=0.5, dilation_iters=2)
    print(f"Grid generated with origin {origin}. Shape: {grid.shape}")
    
    planner = AStarPlanner(grid, origin, resolution=0.5)
    
    start = (0.0, 0.0)
    goal = (10.0, 0.0) # Goal is behind the wall
    
    path = planner.find_path(start, goal)
    
    if path:
        print(f"✅ Path found! {len(path)} waypoints:")
        for wp in path:
            print(f"  {wp[0]:.2f}N, {wp[1]:.2f}E")
    else:
        print("❌ No path found.")

if __name__ == "__main__":
    test_astar_integration()
