# -*- coding: utf-8 -*-
import heapq
import time

class BidirectionalAStar:
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
        def __init__(self, x, y, parent=None, is_forward=True):
            self.x = x
            self.y = y
            self.parent = parent
            self.g = 0
            self.h = 0
            self.f = 0
            self.is_forward = is_forward  # 标记搜索方向

        def __lt__(self, other):
            return self.f < other.f

    def heuristic(self, node, goal):
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return 10 * (dx + dy) + (14 - 2 * 10) * min(dx, dy)

    def find_path(self, start, end):
        start_time = time.time()
        self.nodes_explored = 0
        
        # 初始化正向和反向搜索
        forward_open = []
        backward_open = []
        start_node = self.Node(*start, is_forward=True)
        end_node = self.Node(*end, is_forward=False)
        heapq.heappush(forward_open, start_node)
        heapq.heappush(backward_open, end_node)
        
        forward_closed = dict()
        backward_closed = dict()

        while forward_open and backward_open:
            # 处理正向搜索
            current_forward = heapq.heappop(forward_open)
            self.nodes_explored += 1

            # 检查相遇条件
            if (current_forward.x, current_forward.y) in backward_closed:
                meeting_node = backward_closed[(current_forward.x, current_forward.y)]
                path = self._merge_paths(current_forward, meeting_node)
                self._finalize_stats(start_time, path)
                return path

            if (current_forward.x, current_forward.y) not in forward_closed:
                forward_closed[(current_forward.x, current_forward.y)] = current_forward
                self._expand_node(current_forward, forward_open, forward_closed, backward_closed, is_forward=True, goal=end_node)

            # 处理反向搜索
            current_backward = heapq.heappop(backward_open)
            self.nodes_explored += 1

            # 检查相遇条件
            if (current_backward.x, current_backward.y) in forward_closed:
                meeting_node = forward_closed[(current_backward.x, current_backward.y)]
                path = self._merge_paths(meeting_node, current_backward)
                self._finalize_stats(start_time, path)
                return path

            if (current_backward.x, current_backward.y) not in backward_closed:
                backward_closed[(current_backward.x, current_backward.y)] = current_backward
                self._expand_node(current_backward, backward_open, backward_closed, forward_closed, is_forward=False, goal=start_node)

        self.execution_time = time.time() - start_time
        return None

    def _expand_node(self, current, open_list, closed_dict, other_closed, is_forward, goal):
        for dx, dy in self.movements:
            nx = current.x + dx
            ny = current.y + dy
            if 0 <= nx < self.height and 0 <= ny < self.width:
                # 对角线移动障碍检查
                if dx != 0 and dy != 0:
                    if self.grid[current.x + dx][current.y] == 1 or \
                       self.grid[current.x][current.y + dy] == 1:
                        continue
                
                if self.grid[nx][ny] == 1 or (nx, ny) in closed_dict:
                    continue

                # 计算移动成本
                move_cost = 14 if dx != 0 and dy != 0 else 10
                new_node = self.Node(nx, ny, current, is_forward)
                new_node.g = current.g + move_cost
                new_node.h = self.heuristic(new_node, goal)
                new_node.f = new_node.g + new_node.h

                heapq.heappush(open_list, new_node)

    def _merge_paths(self, forward_node, backward_node):
        # 合并正向和反向路径
        forward_path = []
        current = forward_node
        while current:
            forward_path.append((current.x, current.y))
            current = current.parent

        backward_path = []
        current = backward_node
        while current:
            backward_path.append((current.x, current.y))
            current = current.parent

        # 合并并去除重复点
        return forward_path[::-1] + backward_path[1:]

    def _finalize_stats(self, start_time, path):
        self.execution_time = time.time() - start_time
        if path:
            self.path_length = len(path) 