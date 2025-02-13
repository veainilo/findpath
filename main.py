# -*- coding: utf-8 -*-
from jps import JPS
from astar import AStar
from common import visualize, generate_random_grid, TEST_CONFIG
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # 更新测试用例
    test_cases = [
        ("小型地图", 
         generate_random_grid((10,10), 0.2),  # 参数改为(size, probability)
         (0,0), (9,9)),
        
        ("中型随机地图", 
         generate_random_grid(TEST_CONFIG['medium_map'][:2], TEST_CONFIG['medium_map'][2]),
         (0,0), (TEST_CONFIG['medium_map'][0]-1, TEST_CONFIG['medium_map'][1]-1)),
         
        ("大型随机地图", 
         generate_random_grid(TEST_CONFIG['large_map'][:2], TEST_CONFIG['large_map'][2]),
         (0,0), (TEST_CONFIG['large_map'][0]-1, TEST_CONFIG['large_map'][1]-1)),
         
        ("迷宫地图", 
         generate_random_grid(TEST_CONFIG['maze_map'][:2], TEST_CONFIG['maze_map'][2]),
         (0,0), (TEST_CONFIG['maze_map'][0]-1, TEST_CONFIG['maze_map'][1]-1))
    ]

    # 更新测试输出表头（调整列对齐）
    print(f"{'测试场景':<10} | {'算法':<4} | {'时间(ms)':<8} | {'探索节点':<8} | {'路径长度':<8} | {'平均跳跃':<8} | {'障碍密度':<8}")
    print("-"*85)
    
    for name, grid, start, end in test_cases:
        # 测试JPS
        jps = JPS(grid)
        jps_path = jps.find_path(start, end)
        obstacle_density = sum(row.count(1) for row in grid) / (len(grid) * len(grid[0]))
        print(f"{name:<10} | {'JPS':<4} | {round(jps.execution_time*1000,2):<8} | "
              f"{jps.nodes_explored:<8} | {jps.path_length if jps_path else '无':<8} | "
              f"{round(jps.avg_jump_distance,2) if jps_path else '无':<8} | "
              f"{obstacle_density:<8.2%}")
        
        # 测试A*
        astar = AStar(grid)
        astar_path = astar.find_path(start, end)
        print(f"{name:<10} | {'A*':<4} | {round(astar.execution_time*1000,2):<8} | "
              f"{astar.nodes_explored:<8} | {astar.path_length if astar_path else '无':<8} | "
              f"{'N/A':<8} | "
              f"{obstacle_density:<8.2%}")
        print("-"*85)

    # 更新可视化测试部分
    print("\n运行随机地图测试...")
    random_grid = generate_random_grid((15,15), obstacle_prob=0.25)
    start = (0,0)
    end = (14,14)
    
    # 同时运行两种算法
    jps = JPS(random_grid)
    jps_path = jps.find_path(start, end)
    astar = AStar(random_grid)
    astar_path = astar.find_path(start, end)
    
    # 显示对比结果
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14,6))
    visualize(random_grid, jps_path, 
             title=f"JPS 路径 (耗时: {jps.execution_time*1000:.2f}ms)", 
             stats={'nodes':jps.nodes_explored, 'time':jps.execution_time*1000, 'jumps':jps.jump_calls},
             ax=ax1)
    visualize(random_grid, astar_path,
             title=f"A* 路径 (耗时: {astar.execution_time*1000:.2f}ms)",
             stats={'nodes':astar.nodes_explored, 'time':astar.execution_time*1000},
             ax=ax2)
    plt.show() 