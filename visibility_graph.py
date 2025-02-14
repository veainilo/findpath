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
        
    def get_obstacle_vertices(self):
        """优化后的顶点检测"""
        vertices = []
        # 扩展检测模式，包含更多邻域情况
        patterns = [
            ((-1,0), (0,1)), ((1,0), (0,1)),  # 上下
            ((0,1), (1,0)), ((0,1), (-1,0)),  # 左右
            ((-1,0), (0,-1)), ((1,0), (0,-1)),
            ((0,-1), (1,0)), ((0,-1), (-1,0))
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
        x0, y0 = start
        x1, y1 = end
        
        # 允许起点/终点在障碍物边缘（实际坐标合法即可）
        if not (0 <= x0 < self.height and 0 <= y0 < self.width) or \
           not (0 <= x1 < self.height and 0 <= y1 < self.width):
            return False
        
        line_points = self.bresenham_line(x0, y0, x1, y1)
        for (x, y) in line_points:
            # 只检查路径中间点，不检查起点终点本身
            if (x, y) == start or (x, y) == end:
                continue
            if self.grid[x][y] == 1:
                return False
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
                    edge_count +=1
                
        print(f"可见图边数量: {edge_count}")
        return graph
    
    def find_path(self, start, end):
        """使用Dijkstra算法在可见图中找最短路径"""
        start_time = time.time()
        self.nodes_explored = 0
        
        # 构建可见图
        graph = self.build_visibility_graph(start, end)
        
        # Dijkstra算法
        distances = {vertex: float('infinity') for vertex in graph}
        distances[start] = 0
        pq = [(0, start)]
        previous = {vertex: None for vertex in graph}
        
        while pq:
            self.nodes_explored += 1
            current_distance, current_vertex = heappop(pq)
            
            if current_vertex == end:
                break
                
            if current_distance > distances[current_vertex]:
                continue
            
            for neighbor, weight in graph[current_vertex].items():
                distance = current_distance + weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_vertex
                    heappush(pq, (distance, neighbor))
        
        # 重建路径
        path = []
        current_vertex = end
        while current_vertex is not None:
            path.append(current_vertex)
            current_vertex = previous[current_vertex]
        path.reverse()
        
        self.execution_time = time.time() - start_time
        if path and path[0] == start and path[-1] == end:
            self.path_length = len(path)
            return path
        return None 

    # 临时调试代码：打印顶点数量和示例
    def debug_print(self):
        vertices = self.get_obstacle_vertices()
        print(f"检测到障碍物顶点数量: {len(vertices)}")
        print("示例顶点:", vertices[:5])

    # 在find_path方法中添加路径重建的调试
    def debug_print_path(self, path):
        if path:
            print("找到路径:", path)
        else:
            print("未找到路径，可见图边数量:", sum(len(v) for v in graph.values())) 