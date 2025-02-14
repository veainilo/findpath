# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import colors
import random

# 配置matplotlib中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 简体中文默认字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def visualize(grid, path, title, stats, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 10))
    
    # 绘制路径（更醒目的样式）
    if path:
        ax.plot(
            [p[1] for p in path], 
            [p[0] for p in path], 
            color='#1E90FF',  # 道奇蓝
            linewidth=2.5,
            linestyle='-',
            marker='o',
            markersize=6,
            markerfacecolor='white',
            markeredgewidth=1,
            label='规划路径',
            zorder=2
        )
    
    # 优化统计信息显示
    textstr = '\n'.join([
        f"▸ 探索节点: {stats['nodes']}",
        f"▸ 计算耗时: {stats['time']:.2f} ms",
        f"▸ 路径长度: {len(path) if path else '无'}"
    ])
    
    ax.text(
        0.05, 0.95, 
        textstr, 
        transform=ax.transAxes,
        verticalalignment='top',
        bbox=dict(
            boxstyle='round', 
            facecolor='white', 
            alpha=0.8,
            edgecolor='lightgray'
        ),
        fontsize=11,
        linespacing=1.5
    )
    
    # 设置标题样式
    ax.set_title(
        title, 
        fontsize=14, 
        pad=20, 
        fontweight='bold',
        color='#2F4F4F'
    )
    
    # 设置坐标轴范围
    ax.set_xlim(-0.5, len(grid[0])-0.5)
    ax.set_ylim(len(grid)-0.5, -0.5)  # 反转Y轴保持矩阵显示
    
    # 如果未传入ax则自动显示
    if ax is None:
        plt.show()

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