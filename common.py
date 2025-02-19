# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import colors
import random

# 配置matplotlib中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 简体中文默认字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def visualize(grid, path, title="", stats=None, obstacles=False, ax=None):
    if ax is None:
        fig, ax = plt.subplots()
    
    # 绘制障碍物
    if obstacles:
        for y in range(len(grid)):
            for x in range(len(grid[0])):
                if grid[y][x] == 1:
                    ax.add_patch(plt.Rectangle((x-0.5, y-0.5), 1, 1, color='black'))
    
    # 绘制路径
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, 'r-', linewidth=2)
        ax.plot(xs[0], ys[0], 'go')  # 起点
        ax.plot(xs[-1], ys[-1], 'bx')  # 终点
    
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.grid(True)
    
    # 显示统计信息
    if stats:
        textstr = '\n'.join([f'{k}: {v}' for k, v in stats.items()])
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, 
                verticalalignment='top', bbox=dict(facecolor='white', alpha=0.5))

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