import numpy as np
from visibility_graph import VisibilityGraph
from common import visualize
import matplotlib.pyplot as plt

def create_test_map(size=(20, 20), obstacles=None):
    # 创建一个可变大小的网格
    grid = np.zeros(size, dtype=int)
    
    # 添加障碍物
    if obstacles:
        for obs in obstacles:
            x, y, w, h = obs
            grid[x:x+w, y:y+h] = 1  # 将障碍物区域设置为1

    return grid.tolist()

def test_obstacle_avoidance():
    # 创建不同测试地图的障碍物配置
    obstacles = [
        # 核心阻挡：对角线上的连续障碍墙
        (15, 15, 10, 10),  # 中心大型方块完全阻挡直线
        # 辅助障碍形成必须绕行的通道
        (5, 20, 30, 2),    # 横向障碍墙
        (20, 5, 2, 30),    # 纵向障碍墙
        # 迷宫式障碍
        (10, 10, 5, 2),
        (25, 25, 2, 5),
        (30, 5, 5, 2),
        (5, 30, 2, 5)
    ]
    
    # 创建更大的50x50地图
    grid = create_test_map(size=(50, 50), obstacles=obstacles)

    # 调整起点和终点到对角线两端
    start = (5, 45)  # 左下角附近
    end = (45, 5)    # 右上角附近

    # 使用可见图算法进行路径规划
    vis_graph = VisibilityGraph(grid)
    path = vis_graph.find_path(start, end)

    # 打印找到的路径
    print(f"找到的路径: {path}")

    # 获取算法统计信息
    stats = vis_graph.get_stats()
    print("\n算法统计信息:")
    print(f"探索的节点数: {stats['nodes_explored']}")
    print(f"执行时间: {stats['execution_time']:.4f} 秒")
    print(f"路径长度: {stats['path_length']}")

    # 可视化地图和路径
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='binary')
    
    # 绘制障碍物
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                plt.plot(j, i, 'rs')  # 红色方块表示障碍物
    
    # 绘制起点和终点
    plt.plot(start[1], start[0], 'go', markersize=15, label='起点')  # 绿色圆形表示起点
    plt.plot(end[1], end[0], 'ro', markersize=15, label='终点')      # 红色圆形表示终点
    
    # 如果找到路径，绘制路径
    if path:
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='路径')
    
    plt.grid(True)
    plt.legend()
    plt.title('路径规划结果可视化')
    plt.show()

if __name__ == "__main__":
    test_obstacle_avoidance()