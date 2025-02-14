import numpy as np
import time
from astar import AStar
from jps import JPS
from visibility_graph import VisibilityGraph
import matplotlib.pyplot as plt
from collections import defaultdict

def create_large_sparse_map(size=100, num_obstacles=10, boundary_type="circle"):
    """创建大型稀疏地图"""
    grid = np.zeros((size, size), dtype=int)
    
    # 创建边界
    center = size // 2
    if boundary_type == "circle":
        radius = size * 0.45  # 地图45%大小的圆形边界
        for i in range(size):
            for j in range(size):
                if (i - center) ** 2 + (j - center) ** 2 > radius ** 2:
                    grid[i, j] = 1
    elif boundary_type == "irregular":
        # 创建不规则边界
        from scipy.spatial import ConvexHull
        points = np.random.rand(8, 2) * size
        hull = ConvexHull(points)
        for i in range(size):
            for j in range(size):
                if not point_in_hull((i, j), points[hull.vertices]):
                    grid[i, j] = 1
    
    # 添加随机障碍物
    obstacles_added = 0
    while obstacles_added < num_obstacles:
        x = np.random.randint(size)
        y = np.random.randint(size)
        if grid[x, y] == 0:  # 如果位置为空
            # 添加3x3的障碍物
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    new_x, new_y = x + dx, y + dy
                    if 0 <= new_x < size and 0 <= new_y < size:
                        grid[new_x, new_y] = 1
            obstacles_added += 1
    
    return grid

def point_in_hull(point, hull_points):
    """检查点是否在凸包内"""
    def is_left(p0, p1, p2):
        return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])
    
    n = len(hull_points)
    for i in range(n):
        if is_left(hull_points[i], hull_points[(i+1)%n], point) < 0:
            return False
    return True

def run_pathfinding_test(grid, start, end, algorithm_name, algorithm):
    """运行单次寻路测试"""
    start_time = time.time()
    path = algorithm.find_path(start, end)
    execution_time = time.time() - start_time
    
    return {
        'time': execution_time * 1000,  # 转换为毫秒
        'nodes': algorithm.nodes_explored,
        'path_length': len(path) if path else 0,
        'success': path is not None
    }

def test_pathfinding(num_tests=50):
    """运行多次测试并统计结果"""
    # 测试配置
    configs = [
        {'size': 100, 'obstacles': 5, 'name': '稀疏地图(100x100,5障碍)'},
        {'size': 100, 'obstacles': 20, 'name': '密集地图(100x100,20障碍)'},
        {'size': 200, 'obstacles': 10, 'name': '大地图(200x200,10障碍)'},
    ]
    
    # 存储结果
    results = defaultdict(lambda: defaultdict(list))
    
    for config in configs:
        print(f"\n测试场景: {config['name']}")
        print("-" * 80)
        print(f"{'算法':<15} | {'平均时间(ms)':<12} | {'最短时间':<10} | {'最长时间':<10} | "
              f"{'平均节点':<10} | {'平均路径':<10} | {'成功率':<8}")
        print("-" * 80)
        
        for _ in range(num_tests):
            # 创建地图
            grid = create_large_sparse_map(config['size'], config['obstacles'])
            
            # 生成起终点
            def get_valid_point():
                while True:
                    x = np.random.randint(config['size'])
                    y = np.random.randint(config['size'])
                    if grid[x, y] == 0:
                        return (x, y)
            
            start = get_valid_point()
            end = get_valid_point()
            
            # 测试每个算法
            algorithms = {
                'A*': AStar(grid.tolist()),
                'JPS': JPS(grid.tolist()),
                'Visibility': VisibilityGraph(grid.tolist())
            }
            
            for alg_name, alg in algorithms.items():
                result = run_pathfinding_test(grid, start, end, alg_name, alg)
                for key, value in result.items():
                    results[config['name']][f"{alg_name}_{key}"].append(value)
        
        # 输出该配置下的统计结果
        for alg_name in ['A*', 'JPS', 'Visibility']:
            times = results[config['name']][f"{alg_name}_time"]
            nodes = results[config['name']][f"{alg_name}_nodes"]
            paths = results[config['name']][f"{alg_name}_path_length"]
            successes = results[config['name']][f"{alg_name}_success"]
            
            success_rate = sum(successes) / len(successes) * 100
            
            print(f"{alg_name:<15} | {np.mean(times):>11.2f} | {min(times):>9.2f} | "
                  f"{max(times):>9.2f} | {np.mean(nodes):>9.1f} | "
                  f"{np.mean(paths):>9.1f} | {success_rate:>7.1f}%")

if __name__ == "__main__":
    test_pathfinding() 