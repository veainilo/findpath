# -*- coding: utf-8 -*-
import heapq
import time

class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.movements = [(0, 1), (1, 0), (0, -1), (-1, 0),
                         (1, 1), (1, -1), (-1, 1), (-1, -1)]
        self.nodes_explored = 0
        self.execution_time = 0
        self.path_length = 0

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