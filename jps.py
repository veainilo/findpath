# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import colors
import heapq
import time
import random

# 配置matplotlib中文字体（添加在import之后）
plt.rcParams['font.sans-serif'] = ['SimHei']  # 简体中文默认字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

class JPS:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.movements = [(0, 1), (1, 0), (0, -1), (-1, 0),
                         (1, 1), (1, -1), (-1, 1), (-1, -1)]
        self.nodes_explored = 0
        self.jump_calls = 0
        self.execution_time = 0
        self.jump_cache = {}  # 新增跳跃点缓存
        self.path_length = 0  # 新增路径长度统计
        self.avg_jump_distance = 0  # 平均跳跃距离

    class Node:
        def __init__(self, x, y, parent=None):
            self.x = x
            self.y = y
            self.parent = parent
            self.g = 0
            self.h = 0
            self.f = 0

        def __lt__(self, other):
            # 应该优先比较f值，再比较h值
            return (self.f, self.h) < (other.f, other.h)  # 添加次级比较条件

    def heuristic(self, node, goal):
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return 10 * (dx + dy) + (14 - 20) * min(dx, dy)  # 修正后的Octile距离

    def jump(self, x, y, dx, dy, goal):
        cache_key = (x, y, dx, dy)
        if cache_key in self.jump_cache:
            return self.jump_cache[cache_key]
            
        while True:
            x += dx
            y += dy
            # 越界或障碍物检查
            if not (0 <= x < self.height and 0 <= y < self.width) or self.grid[x][y] == 1:
                return None
            
            # 到达终点检查
            if (x, y) == (goal.x, goal.y):
                return (x, y)
            
            # 强制邻居检测（优化后的版本）
            if self.has_forced_neighbor(x, y, dx, dy):
                return (x, y)
            
            # 对角线移动时检查直线方向
            if dx != 0 and dy != 0:
                if self.jump(x, y, dx, 0, goal) or self.jump(x, y, 0, dy, goal):
                    return (x, y)

        # 缓存结果
        self.jump_cache[cache_key] = result
        return result

    def find_path(self, start, end):
        start_time = time.time()
        self.nodes_explored = 0
        self.jump_calls = 0
        
        open_list = []
        start_node = self.Node(*start)
        end_node = self.Node(*end)
        heapq.heappush(open_list, start_node)
        
        closed_list = set()
        g_values = {start_node: 0}  # 新增g值缓存
        
        while open_list:
            current = heapq.heappop(open_list)
            # 跳过已处理的节点（修复重复扩展问题）
            if current.g > g_values.get((current.x, current.y), float('inf')):
                continue
            
            self.nodes_explored += 1

            if current.x == end_node.x and current.y == end_node.y:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                self.execution_time = time.time() - start_time
                if path:
                    self.path_length = len(path)
                    total_jumps = sum(max(abs(path[i][0]-path[i-1][0]), abs(path[i][1]-path[i-1][1])) 
                                   for i in range(1, len(path)))
                    self.avg_jump_distance = total_jumps / (len(path)-1) if len(path)>1 else 0
                    return self.smooth_path(path)
                return None

            closed_list.add((current.x, current.y))
            
            neighbors = []
            for dx, dy in self.movements:
                jump_point = self.jump(current.x, current.y, dx, dy, end_node)
                if jump_point:
                    # 修正后的成本计算（原计算方式导致估值错误）
                    step_x = jump_point[0] - current.x
                    step_y = jump_point[1] - current.y
                    distance = max(abs(step_x), abs(step_y))  # 正确计算对角线步数
                    cost = 14 * distance if dx * dy != 0 else 10 * distance
                    neighbors.append((jump_point, cost))

            for neighbor, cost in neighbors:
                nx, ny = neighbor
                if (nx, ny) in closed_list:
                    continue
                
                new_g = current.g + cost
                # 仅当新路径更优时更新
                if new_g < g_values.get((nx, ny), float('inf')):
                    new_node = self.Node(nx, ny, current)
                    new_node.g = new_g
                    new_node.h = self.heuristic(new_node, end_node)
                    new_node.f = new_node.g + new_node.h
                    g_values[(nx, ny)] = new_g
                    heapq.heappush(open_list, new_node)

        self.execution_time = time.time() - start_time
        return None

    def smooth_path(self, path):
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            # 检查直线可达性
            if not self.has_obstacle(smoothed[-1], path[i+1]):
                continue
            smoothed.append(path[i])
        smoothed.append(path[-1])
        return smoothed

    def has_obstacle(self, start, end):
        # Bresenham直线算法检查障碍物
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            if self.grid[x0][y0] == 1:
                return True
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return False

    def has_forced_neighbor(self, x, y, dx, dy):
        # 水平/垂直移动
        if dx == 0 or dy == 0:
            # 垂直移动时检查两侧
            if dx == 0:
                if (self.is_blocked(x+1, y) and not self.is_blocked(x, y+dy)) or \
                   (self.is_blocked(x-1, y) and not self.is_blocked(x, y+dy)):
                    return True
            # 水平移动同理
        else:  # 对角线移动
            if (self.is_blocked(x-dx, y) and not self.is_blocked(x, y+dy)) or \
               (self.is_blocked(x, y-dy) and not self.is_blocked(x+dx, y)):
                return True
        return False

    def is_blocked(self, x, y):
        return not (0 <= x < self.height and 0 <= y < self.width) or self.grid[x][y] == 1

# 在JPS类之后添加A*算法实现
class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.movements = [(0, 1), (1, 0), (0, -1), (-1, 0),
                         (1, 1), (1, -1), (-1, 1), (-1, -1)]
        self.nodes_explored = 0
        self.execution_time = 0
        self.path_length = 0  # 新增路径长度统计

    class Node:
        def __init__(self, x, y, parent=None):
            self.x = x
            self.y = y
            self.parent = parent
            self.g = 0
            self.h = 0
            self.f = 0

        def __lt__(self, other):
            return self.f < other.f

    def heuristic(self, node, goal):
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return 10 * (dx + dy) + (14 - 2 * 10) * min(dx, dy)

    def find_path(self, start, end):
        start_time = time.time()
        self.nodes_explored = 0
        
        open_list = []
        start_node = self.Node(*start)
        end_node = self.Node(*end)
        heapq.heappush(open_list, start_node)
        
        closed_dict = dict()
        
        while open_list:
            current = heapq.heappop(open_list)
            self.nodes_explored += 1

            if current.x == end_node.x and current.y == end_node.y:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                self.execution_time = time.time() - start_time
                if path:
                    self.path_length = len(path)
                return path[::-1]

            if (current.x, current.y) in closed_dict:
                continue
            closed_dict[(current.x, current.y)] = current

            for dx, dy in self.movements:
                nx = current.x + dx
                ny = current.y + dy
                if 0 <= nx < self.height and 0 <= ny < self.width:
                    # 添加对角线移动的障碍物检查
                    if dx != 0 and dy != 0:  # 对角线移动
                        # 检查水平方向和垂直方向是否可通行
                        if self.grid[current.x + dx][current.y] == 1 or \
                           self.grid[current.x][current.y + dy] == 1:
                            continue
                    
                    if self.grid[nx][ny] == 1 or (nx, ny) in closed_dict:
                        continue
                    
                    # 计算移动成本
                    if dx != 0 and dy != 0:  # 对角线移动
                        move_cost = 14
                    else:
                        move_cost = 10
                        
                    new_node = self.Node(nx, ny, current)
                    new_node.g = current.g + move_cost
                    new_node.h = self.heuristic(new_node, end_node)
                    new_node.f = new_node.g + new_node.h
                    
                    heapq.heappush(open_list, new_node)

        self.execution_time = time.time() - start_time
        return None

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
    
    # 添加带默认值的统计信息
    if stats:
        stats_text = f"探索节点: {stats.get('nodes', 'N/A')}\n耗时: {stats.get('time', 'N/A')}ms\n跳跃次数: {stats.get('jumps', 'N/A')}"
        ax.text(0.5, -0.15, stats_text, transform=ax.transAxes, ha='center', fontsize=9)
    
    plt.tight_layout()
    return ax

# 添加随机障碍物生成函数
def generate_random_grid(size, obstacle_prob=0.2):
    """生成保证起点终点连通的随机网格"""
    while True:
        grid = [[1 if random.random() < obstacle_prob and (i,j) not in [(0,0),(size[1]-1,size[0]-1)] else 0 
                for j in range(size[0])] for i in range(size[1])]
        # 简单连通性检查
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

# 修改测试用例部分
if __name__ == "__main__":
    # 随机测试参数配置
    TEST_CONFIG = {
        'small_map': (10, 10, 0.2),
        'medium_map': (20, 20, 0.3),
        'large_map': (30, 30, 0.25),
        'maze_map': (40, 40, 0.28)
    }

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
