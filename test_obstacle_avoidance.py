import numpy as np
from visibility_graph import VisibilityGraph
from common import visualize
import matplotlib.pyplot as plt

def create_test_map():
    # 创建一个20x20的网格
    grid = np.zeros((20, 20), dtype=int)
    
    # 添加一些障碍物
    # 添加一个L形障碍物
    for i in range(8, 12):
        grid[i][10] = 1  # 垂直线
    for j in range(10, 15):
        grid[11][j] = 1  # 水平线
        
    # 添加一个小方块障碍物
    for i in range(5, 7):
        for j in range(5, 7):
            grid[i][j] = 1
            
    return grid.tolist()

def test_obstacle_avoidance():
    # 创建测试地图
    grid = create_test_map()
    
    # 设置起点和终点
    start = (5, 15)
    end = (15, 5)
    
    # 使用可见图算法进行路径规划
    vis_graph = VisibilityGraph(grid)
    path = vis_graph.find_path(start, end)
    
    # 优化后的可视化部分
    plt.figure(figsize=(10, 10), dpi=100)
    ax = plt.gca()
    
    # 绘制障碍物（更醒目的样式）
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                ax.add_patch(plt.Rectangle(
                    (j-0.5, i-0.5), 1, 1, 
                    color='#2F4F4F',  # 深石板灰
                    ec='#696969',     # 暗灰色边框
                    lw=0.5,
                    alpha=0.7
                ))
    
    # 绘制顶点标记（更清晰的样式）
    obstacle_vertices = vis_graph.get_obstacle_vertices()
    ax.scatter(
        [v[1] for v in obstacle_vertices], 
        [v[0] for v in obstacle_vertices], 
        c='#FF4500',    # 橙红色
        s=50, 
        marker='D',     # 菱形标记
        edgecolor='k',
        label='障碍物顶点',
        zorder=3        # 确保在最上层
    )
    
    # 调用可视化函数
    stats = {
        'nodes': vis_graph.nodes_explored,
        'time': vis_graph.execution_time * 1000,
    }
    visualize(grid, path, "绕障路径可视化", stats, ax=ax)
    
    # 添加图例和坐标标签
    ax.legend(
        loc='upper right',
        fontsize=10,
        framealpha=0.9,
        edgecolor='w'
    )
    ax.set_xlabel('X 坐标', fontsize=12)
    ax.set_ylabel('Y 坐标', fontsize=12)
    
    # 设置网格样式
    ax.grid(True, color='lightgray', linestyle='--', linewidth=0.5)
    ax.set_xticks(range(0, 20, 1))
    ax.set_yticks(range(0, 20, 1))
    ax.tick_params(axis='both', which='both', length=0)  # 隐藏刻度线
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    test_obstacle_avoidance() 