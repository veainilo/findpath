import numpy as np
from heapq import heappush, heappop
import time

class VisibilityGraph:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.nodes_explored = 0
        self.execution_time = 0
        self.path_length = 0
        self.visibility_cache = {}  # 可见性缓存

    def get_obstacle_vertices(self):
        """优化后的顶点检测"""
        vertices = []
        # 优化检测模式，减少重复
        patterns = [
            ((-1,0), (0,1)),  # 左上
            ((1,0), (0,1)),   # 左下
            ((0,1), (1,0)),   # 右下
            ((0,-1), (-1,0))  # 右上
        ]
        
        for i in range(self.height):
            for j in range(self.width):
                if self.grid[i][j] == 1:
                    for (dx1, dy1), (dx2, dy2) in patterns:
                        p1 = (i + dx1, j + dy1)
                        p2 = (i + dx2, j + dy2)
                        if self.is_valid_empty(p1) or self.is_valid_empty(p2):
                            vertices.append((i, j))
                            break
        return vertices
    
    def is_valid_empty(self, pos):
        x, y = pos
        if 0 <= x < self.height and 0 <= y < self.width:
            return self.grid[x][y] == 0
        return True  # 边界外视为可通行
    
    def is_visible(self, start, end):
        """改进的可见性检查"""
        cache_key = (start, end)
        if cache_key in self.visibility_cache:
            return self.visibility_cache[cache_key]
        
        x0, y0 = start
        x1, y1 = end
        
        if not (0 <= x0 < self.height and 0 <= y0 < self.width) or \
           not (0 <= x1 < self.height and 0 <= y1 < self.width):
            self.visibility_cache[cache_key] = False
            return False
        
        line_points = self.bresenham_line(x0, y0, x1, y1)
        for (x, y) in line_points:
            if (x, y) == start or (x, y) == end:
                continue
            if self.grid[x][y] == 1:
                self.visibility_cache[cache_key] = False
                return False
                
        self.visibility_cache[cache_key] = True
        return True
    
    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points
    
    def build_visibility_graph(self, start, end):
        """构建可见图"""
        vertices = self.get_obstacle_vertices()
        print(f"障碍物顶点数量: {len(vertices)}")
        
        vertices.append(start)
        vertices.append(end)
        
        graph = {v: {} for v in vertices}
        edge_count = 0
        
        for i, v1 in enumerate(vertices):
            for v2 in vertices[i+1:]:
                if self.is_visible(v1, v2):
                    dist = ((v1[0]-v2[0])**2 + (v1[1]-v2[1])**2)**0.5
                    graph[v1][v2] = dist
                    graph[v2][v1] = dist
                    edge_count += 1
                    
        print(f"可见图边数量: {edge_count}")
        return graph
    
    def find_path(self, start, end):
        """使用A*算法在可见图中找最短路径"""
        print(f"起点有效性: {self.is_valid_empty(start)}")
        print(f"终点有效性: {self.is_valid_empty(end)}")
        start_time = time.time()
        self.nodes_explored = 0
        self.visibility_cache = {}  # 清空缓存
        
        # 构建可见图
        graph = self.build_visibility_graph(start, end)
        
        print("开始路径寻找...")
        print("构建的可见图:", graph)
        
        # A*算法
        g_score = {vertex: float('infinity') for vertex in graph}
        g_score[start] = 0
        f_score = {vertex: float('infinity') for vertex in graph}
        f_score[start] = self.heuristic(start, end)
        
        open_set = [(f_score[start], start)]
        came_from = {}
        
        while open_set:
            self.nodes_explored += 1
            current = heappop(open_set)[1]
            
            if current == end:
                path = self.reconstruct_path(came_from, end)
                self.execution_time = time.time() - start_time
                self.path_length = len(path)
                return path
            
            for neighbor, cost in graph[current].items():
                tentative_g_score = g_score[current] + cost
                
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, end)
                    heappush(open_set, (f_score[neighbor], neighbor))
        
        self.execution_time = time.time() - start_time
        return None

    def heuristic(self, a, b):
        """启发式函数：欧几里得距离"""
        return 1.1 * ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def reconstruct_path(self, came_from, current):
        """重建路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def get_stats(self):
        """获取算法统计信息"""
        return {
            'nodes_explored': self.nodes_explored,
            'execution_time': self.execution_time,
            'path_length': self.path_length,
            'cache_hits': len(self.visibility_cache) - sum(self.visibility_cache.values())
        }
