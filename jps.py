# -*- coding: utf-8 -*-
import heapq
import time

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
        self.jump_cache = {}  # 跳跃点缓存
        self.path_length = 0  # 路径长度统计
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
        g_values = {start_node: 0}  # g值缓存
        
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
