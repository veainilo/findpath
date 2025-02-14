# -*- coding: utf-8 -*-
from jps import JPS
from astar import AStar
from SimplePathFinderNew import SimplePathFinder
from common import visualize, generate_random_grid, TEST_CONFIG
import matplotlib.pyplot as plt
import time
import numpy as np
from bidirectional_astar import BidirectionalAStar

# 地图配置参数
MAP_CONFIG = {
    'small': (10, 10, 0.2),
    'medium': TEST_CONFIG['medium_map'],  # (30,30,0.15)
    'large': TEST_CONFIG['large_map'],    # (50,50,0.2)
    'maze': TEST_CONFIG['maze_map'],      # (40,40,0.3)
    'random_demo': (20, 20, 0.25)       # 新增演示用随机地图配置
}

def create_test_grid(width, height, obstacle_density=0.3):
    """创建测试用的网格地图"""
    grid = np.random.choice([0, 1], size=(height, width), p=[1-obstacle_density, obstacle_density])
    # 确保起点和终点可用
    grid[0, 0] = 0
    grid[-1, -1] = 0
    return grid

def test_pathfinding(grid, start, end):
    """测试并比较两种寻路算法的性能"""
    # 测试A*
    astar = AStar(grid)
    start_time = time.time()
    path_astar = astar.find_path(start, end)
    astar_time = astar.execution_time
    
    # 测试双向A*
    bi_astar = BidirectionalAStar(grid)
    start_time = time.time()
    path_bi = bi_astar.find_path(start, end)
    bi_astar_time = bi_astar.execution_time
    
    # 打印结果
    print("\n性能比较结果:")
    print("-" * 50)
    print(f"地图大小: {grid.shape[0]}x{grid.shape[1]}")
    
    print("\nA*算法:")
    print(f"执行时间: {astar_time:.6f}秒")
    print(f"探索节点数: {astar.nodes_explored}")
    if path_astar:
        print(f"路径长度: {astar.path_length}")
    else:
        print("未找到路径")
    
    print("\n双向A*算法:")
    print(f"执行时间: {bi_astar_time:.6f}秒")
    print(f"探索节点数: {bi_astar.nodes_explored}")
    if path_bi:
        print(f"路径长度: {bi_astar.path_length}")
    else:
        print("未找到路径")
    
    if path_astar and path_bi:
        print("\n路径长度比较:")
        print(f"A*路径长度: {len(path_astar)}")
        print(f"双向A*路径长度: {len(path_bi)}")
    
    print("\n性能提升:")
    if astar_time > 0:
        speedup = (astar_time - bi_astar_time) / astar_time * 100
        print(f"时间提升: {speedup:.2f}%")
    
    nodes_reduction = (astar.nodes_explored - bi_astar.nodes_explored) / astar.nodes_explored * 100
    print(f"节点探索减少: {nodes_reduction:.2f}%")
    
    return path_astar, path_bi

def main():
    # 测试不同大小的地图
    map_sizes = [(50, 50), (100, 100), (200, 200)]
    obstacle_densities = [0.2, 0.3]
    
    for size in map_sizes:
        for density in obstacle_densities:
            print(f"\n测试地图 {size[0]}x{size[1]}, 障碍物密度: {density}")
            print("=" * 60)
            
            grid = create_test_grid(size[0], size[1], density)
            start = (0, 0)
            end = (size[0]-1, size[1]-1)
            
            path_astar, path_bi = test_pathfinding(grid, start, end)

    # 更新测试用例
    test_cases = [
        ("小型地图", 
         generate_random_grid(MAP_CONFIG['small'][:2], MAP_CONFIG['small'][2]),
         (0,0), (MAP_CONFIG['small'][0]-1, MAP_CONFIG['small'][1]-1)),
        
        ("中型地图", 
         generate_random_grid(MAP_CONFIG['medium'][:2], MAP_CONFIG['medium'][2]),
         (0,0), (MAP_CONFIG['medium'][0]-1, MAP_CONFIG['medium'][1]-1)),
         
        ("大型地图", 
         generate_random_grid(MAP_CONFIG['large'][:2], MAP_CONFIG['large'][2]),
         (0,0), (MAP_CONFIG['large'][0]-1, MAP_CONFIG['large'][1]-1)),
         
        ("迷宫地图", 
         generate_random_grid(MAP_CONFIG['maze'][:2], MAP_CONFIG['maze'][2]),
         (0,0), (MAP_CONFIG['maze'][0]-1, MAP_CONFIG['maze'][1]-1))
    ]

    # 更新表头增加SPF列
    print(f"{'测试场景':<10} | {'算法':<6} | {'时间(ms)':<8} | {'探索节点':<8} | {'路径长度':<8} | {'平均跳跃':<8} | {'障碍密度':<8}")
    print("-"*90)
    
    for name, grid, start, end in test_cases:
        obstacle_density = sum(row.count(1) for row in grid) / (len(grid) * len(grid[0]))
        
        # 测试SimplePathFinder
        spf = SimplePathFinder(grid)
        start_time = time.time()
        spf_path = spf.find_path(start, end)
        spf_time = (time.time() - start_time) * 1000
        spf_nodes = len(spf.visited) if spf_path else 0
        spf_length = len(spf_path) if spf_path else '无'
        print(f"{name:<10} | {'SPF':<6} | {spf_time:<8.2f} | "
              f"{spf_nodes:<8} | {spf_length:<8} | "
              f"{'N/A':<8} | "
              f"{obstacle_density:<8.2%}")

        # 测试JPS
        jps = JPS(grid)
        jps_path = jps.find_path(start, end)
        print(f"{name:<10} | {'JPS':<6} | {round(jps.execution_time*1000,2):<8} | "
              f"{jps.nodes_explored:<8} | {jps.path_length if jps_path else '无':<8} | "
              f"{round(jps.avg_jump_distance,2) if jps_path else '无':<8} | "
              f"{obstacle_density:<8.2%}")
        
        # 测试A*
        astar = AStar(grid)
        astar_path = astar.find_path(start, end)
        print(f"{name:<10} | {'A*':<6} | {round(astar.execution_time*1000,2):<8} | "
              f"{astar.nodes_explored:<8} | {astar.path_length if astar_path else '无':<8} | "
              f"{'N/A':<8} | "
              f"{obstacle_density:<8.2%}")
        
        # 新增双向A*测试
        bi_astar = BidirectionalAStar(grid)
        bi_astar_path = bi_astar.find_path(start, end)
        print(f"{name:<10} | {'BiA*':<6} | {round(bi_astar.execution_time*1000,2):<8} | "
              f"{bi_astar.nodes_explored:<8} | {bi_astar.path_length if bi_astar_path else '无':<8} | "
              f"{'N/A':<8} | "
              f"{obstacle_density:<8.2%}")

        print("-"*90)

    # 更新可视化部分
    print("\n运行随机地图测试...")
    random_grid = generate_random_grid(
        MAP_CONFIG['random_demo'][:2], 
        obstacle_prob=MAP_CONFIG['random_demo'][2]
    )
    start = (0, 0)
    end = (MAP_CONFIG['random_demo'][0]-1, MAP_CONFIG['random_demo'][1]-1)

    # 运行所有算法
    spf = SimplePathFinder(random_grid)
    start_time = time.time()
    spf_path = spf.find_path(start, end)
    spf_time = (time.time() - start_time) * 1000

    jps = JPS(random_grid)
    jps_path = jps.find_path(start, end)

    astar = AStar(random_grid)
    astar_path = astar.find_path(start, end)

    bi_astar = BidirectionalAStar(random_grid)
    bi_astar_path = bi_astar.find_path(start, end)

    # 调整可视化布局为2x2
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18,12))

    # 可视化所有算法的路径
    visualize(random_grid, spf_path,
             title=f"SPF 路径 (耗时: {spf_time:.2f}ms)",
             stats={'nodes':len(spf.visited), 'time':spf_time},
             obstacles=True,
             ax=ax1)

    visualize(random_grid, jps_path,
             title=f"JPS 路径 (耗时: {jps.execution_time*1000:.2f}ms)",
             stats={'nodes':jps.nodes_explored, 'time':jps.execution_time*1000},
             obstacles=True,
             ax=ax2)

    visualize(random_grid, astar_path,
             title=f"A* 路径 (耗时: {astar.execution_time*1000:.2f}ms)",
             stats={'nodes':astar.nodes_explored, 'time':astar.execution_time*1000},
             obstacles=True,
             ax=ax3)

    visualize(random_grid, bi_astar_path,
             title=f"双向A* 路径 (耗时: {bi_astar.execution_time*1000:.2f}ms)",
             stats={'nodes':bi_astar.nodes_explored, 'time':bi_astar.execution_time*1000},
             obstacles=True,
             ax=ax4)

    plt.tight_layout()  # 调整子图布局,防止重叠
    plt.show()

if __name__ == "__main__":
    main() 