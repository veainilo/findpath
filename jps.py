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
            
        self.jump_calls += 1
        nx, ny = x + dx, y + dy
        
        # 越界或障碍物检查
        if not (0 <= nx < self.height and 0 <= ny < self.width) or self.grid[nx][ny] == 1:
            self.jump_cache[cache_key] = None
            return None
            
        # 到达终点检查
        if (nx, ny) == (goal.x, goal.y):
            self.jump_cache[cache_key] = (nx, ny)
            return (nx, ny)
            
        # 强制邻居检测
        if self.has_forced_neighbor(nx, ny, dx, dy):
            self.jump_cache[cache_key] = (nx, ny)
            return (nx, ny)
            
        # 对角线移动时的优化检查
        if dx != 0 and dy != 0:
            # 只在必要时检查水平和垂直方向
            if (self.is_blocked(nx-dx, ny) and not self.is_blocked(nx, ny+dy)) or \
               (self.is_blocked(nx, ny-dy) and not self.is_blocked(nx+dx, ny)):
                self.jump_cache[cache_key] = (nx, ny)
                return (nx, ny)
                
        # 继续跳点搜索
        next_point = self.jump(nx, ny, dx, dy, goal)
        self.jump_cache[cache_key] = next_point
        return next_point

    def find_path(self, start, end):
        start_time = time.time()
        self.nodes_explored = 0
        self.jump_calls = 0
        
        open_list = []
        start_node = self.Node(*start)
        end_node = self.Node(*end)
        start_node.h = self.heuristic(start_node, end_node)
        start_node.f = start_node.h
        heapq.heappush(open_list, start_node)
        
        closed_dict = {}
        g_values = {start: 0}
        
        # 动态调整最大迭代次数
        max_iterations = min(self.height * self.width // 2, 10000)
        iterations = 0
        
        # 计算主要方向
        dx_main = 1 if end_node.x > start_node.x else -1 if end_node.x < start_node.x else 0
        dy_main = 1 if end_node.y > start_node.y else -1 if end_node.y < start_node.y else 0
        
        # 优化移动方向顺序
        primary_moves = []
        if dx_main != 0 and dy_main != 0:
            primary_moves.append((dx_main, dy_main))
        if dx_main != 0:
            primary_moves.append((dx_main, 0))
        if dy_main != 0:
            primary_moves.append((0, dy_main))
            
        # 添加次要方向
        secondary_moves = [(dx, dy) for dx, dy in self.movements if (dx, dy) not in primary_moves]
        optimized_movements = primary_moves + secondary_moves
        
        while open_list and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_list)
            current_pos = (current.x, current.y)
            
            if current_pos == (end_node.x, end_node.y):
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
                    return self.smooth_path(path[::-1])
                return None

            if current_pos in closed_dict:
                continue
                
            self.nodes_explored += 1
            closed_dict[current_pos] = current

            # 使用优化后的移动方向
            successors = []
            for dx, dy in optimized_movements:
                jump_point = self.jump(current.x, current.y, dx, dy, end_node)
                if jump_point:
                    nx, ny = jump_point
                    if (nx, ny) not in closed_dict:
                        dx_total = nx - current.x
                        dy_total = ny - current.y
                        
                        # 优化代价计算
                        if dx_total != 0 and dy_total != 0:
                            diagonal_steps = min(abs(dx_total), abs(dy_total))
                            straight_steps = max(abs(dx_total), abs(dy_total)) - diagonal_steps
                            cost = 14 * diagonal_steps + 10 * straight_steps
                        else:
                            cost = 10 * (abs(dx_total) + abs(dy_total))
                            
                        # 计算到目标的启发值
                        h_value = self.heuristic(self.Node(nx, ny), end_node)
                        successors.append((jump_point, cost, h_value))

            # 按照f值（g+h）对后继节点进行排序
            successors.sort(key=lambda x: current.g + x[1] + x[2])
            
            for (nx, ny), cost, _ in successors:
                new_g = current.g + cost
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
        if len(path) <= 2:
            return path
            
        smoothed = [path[0]]
        current = 0
        
        while current < len(path) - 1:
            # 尝试找到最远的可直接到达的点
            for i in range(len(path)-1, current, -1):
                if not self.has_obstacle(path[current], path[i]):
                    smoothed.append(path[i])
                    current = i
                    break
            else:
                current += 1
                if current < len(path):
                    smoothed.append(path[current])
                    
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
