# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import colors
import random

# 配置matplotlib中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 简体中文默认字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def visualize(grid, path=None, title="", stats=None, ax=None):
    cmap = colors.ListedColormap(['white', 'black', 'red', 'green', 'blue'])
    if ax is None:
        fig, ax = plt.subplots()
    ax.imshow(grid, cmap=cmap)
    
    if path:
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        ax.plot(path_x, path_y, c='blue', linewidth=2)
    
    legend = [
        mpatches.Patch(color='white', label='可行区域'),
        mpatches.Patch(color='black', label='障碍物'),
        mpatches.Patch(color='blue', label='路径')
    ]
    ax.legend(handles=legend, bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.set_title(f"{title}\n路径长度: {len(path) if path else '无'}", fontsize=10)
    
    if stats:
        stats_text = f"探索节点: {stats.get('nodes', 'N/A')}\n耗时: {stats.get('time', 'N/A')}ms\n跳跃次数: {stats.get('jumps', 'N/A')}"
        ax.text(0.5, -0.15, stats_text, transform=ax.transAxes, ha='center', fontsize=9)
    
    plt.tight_layout()
    return ax

def generate_random_grid(size, obstacle_prob=0.2):
    """生成保证起点终点连通的随机网格"""
    while True:
        grid = [[1 if random.random() < obstacle_prob and (i,j) not in [(0,0),(size[1]-1,size[0]-1)] else 0 
                for j in range(size[0])] for i in range(size[1])]
        if bfs((0,0), (size[1]-1, size[0]-1), grid):
            return grid

def bfs(start, end, grid):
    """广度优先搜索验证连通性"""
    queue = [start]
    visited = set()
    while queue:
        x, y = queue.pop(0)
        if (x, y) == end:
            return True
        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            nx, ny = x+dx, y+dy
            if 0<=nx<len(grid) and 0<=ny<len(grid[0]) and grid[nx][ny]==0 and (nx,ny) not in visited:
                visited.add((nx,ny))
                queue.append((nx,ny))
    return False

# 测试配置
TEST_CONFIG = {
    'small_map': (20, 20, 0.2),
    'medium_map': (50, 50, 0.3),
    'large_map': (100, 100, 0.25),
    'maze_map': (200, 200, 0.28)
} 