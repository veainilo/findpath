import heapq
from math import sqrt
import time

class JPS:
    class Node:
        def __init__(self, x, y, parent=None):
            self.x = x
            self.y = y
            self.parent = parent
            self.g = 0  # 实际成本
            self.h = 0  # 启发式估计
            self.f = 0  # g + h

        def __lt__(self, other):
            return self.f < other.f

    def __init__(self, grid):
        self.grid = grid
        self.execution_time = 0
        self.nodes_explored = 0
        self.path_length = 0
        self.avg_jump_distance = 0

    @staticmethod
    def heuristic(a, b):
        # 使用对角线距离（允许8方向移动时更准确）
        dx = abs(a.x - b.x)
        dy = abs(a.y - b.y)
        # 修改启发式函数的权重，使其更倾向于对角线移动
        return max(dx, dy) * 10 + min(dx, dy) * 4  # 对角线移动的权重降低

    def find_path(self, start, end):
        start_time = time.time()
        path = self._jps_search(start, end)
        self.execution_time = time.time() - start_time
        
        if path:
            self.path_length = len(path)
            jump_distances = [sqrt((x1-x0)**2 + (y1-y0)**2) 
                            for (x0,y0),(x1,y1) in zip(path[:-1], path[1:])]
            self.avg_jump_distance = sum(jump_distances)/len(jump_distances) if jump_distances else 0
            self.nodes_explored = len(set((x, y) for x, y in path))
        return path

    def _jps_search(self, start, end):
        open_list = []
        closed_set = set()

        start_node = self.Node(*start)
        end_node = self.Node(*end)
        start_node.h = self.heuristic(start_node, end_node)
        start_node.f = start_node.h

        heapq.heappush(open_list, start_node)
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if (current.x, current.y) == (end_node.x, end_node.y):
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]

            closed_set.add((current.x, current.y))
            
            neighbors = self._get_neighbors(current, end_node)
            
            for nx, ny, direction in neighbors:
                if (nx, ny) in closed_set:
                    continue
                    
                dx, dy = direction
                jump_point = self._jump(current.x, current.y, dx, dy, end_node)
                
                if not jump_point:
                    continue

                jx, jy = jump_point
                if self.grid[jx][jy] == 1:
                    continue

                jump_node = self.Node(jx, jy, current)
                cost = 14 if dx !=0 and dy !=0 else 10
                jump_node.g = current.g + cost * sqrt((jx-current.x)**2 + (jy-current.y)**2)
                jump_node.h = self.heuristic(jump_node, end_node)
                jump_node.f = jump_node.g + jump_node.h

                found = False
                for i, node in enumerate(open_list):
                    if (node.x, node.y) == (jx, jy):
                        found = True
                        if node.g > jump_node.g:
                            open_list[i] = jump_node
                            heapq.heapify(open_list)
                        break
                if not found:
                    heapq.heappush(open_list, jump_node)

        return None

    def _get_neighbors(self, current, end_node):
        if not current.parent:  # 起点检查所有方向
            print("处理起点")
            neighbors = []
            # 优先检查对角线方向
            directions = [(1, 1), (1, -1), (-1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)]
            for dx, dy in directions:
                nx, ny = current.x + dx, current.y + dy
                if 0 <= nx < len(self.grid) and 0 <= ny < len(self.grid[0]):
                    # 对于所有方向，只要目标格可通行就允许移动
                    if self.grid[nx][ny] == 0:
                        if dx != 0 and dy != 0:
                            print(f"添加起点对角线邻居: ({nx}, {ny})")
                        else:
                            print(f"添加起点直线邻居: ({nx}, {ny})")
                        neighbors.append((nx, ny, (dx, dy)))
            return neighbors

        # 根据父节点确定搜索方向
        neighbors = []
        x, y = current.x, current.y

        # 调试信息
        print(f"检查节点 ({x}, {y}) 的邻居")

        # 检查是否可以直接到达目标点
        if abs(end_node.x - x) <= 1 and abs(end_node.y - y) <= 1:
            if self.grid[end_node.x][end_node.y] == 0:
                dx = end_node.x - x
                dy = end_node.y - y
                if dx != 0 and dy != 0:  # 对角线移动到目标
                    if self.grid[x][end_node.y] == 0 and self.grid[end_node.x][y] == 0:
                        print(f"添加目标点邻居: ({end_node.x}, {end_node.y})")
                        neighbors.append((end_node.x, end_node.y, (dx, dy)))
                else:  # 直线移动到目标
                    print(f"添加目标点邻居: ({end_node.x}, {end_node.y})")
                    neighbors.append((end_node.x, end_node.y, (dx, dy)))

        # 计算相对父节点的方向
        dx = current.x - current.parent.x
        dy = current.y - current.parent.y
        dx = 1 if dx > 0 else (-1 if dx < 0 else 0)
        dy = 1 if dy > 0 else (-1 if dy < 0 else 0)

        print(f"移动方向: dx={dx}, dy={dy}")

        # 自然邻居（沿原方向）
        if dx != 0 and dy != 0:  # 对角线移动
            print("处理对角线移动")
            # 检查对角线方向是否可行，放宽条件
            if (0 <= x + dx < len(self.grid) and 0 <= y + dy < len(self.grid[0]) and 
                self.grid[x + dx][y + dy] == 0):  # 只检查目标格是否可通行
                print(f"添加对角线邻居: ({x + dx}, {y + dy})")
                neighbors.append((x + dx, y + dy, (dx, dy)))
                
            # 总是检查水平和垂直方向
            if 0 <= x + dx < len(self.grid) and self.grid[x + dx][y] == 0:
                print(f"添加水平邻居: ({x + dx}, {y})")
                neighbors.append((x + dx, y, (dx, 0)))
            if 0 <= y + dy < len(self.grid[0]) and self.grid[x][y + dy] == 0:
                print(f"添加垂直邻居: ({x}, {y + dy})")
                neighbors.append((x, y + dy, (0, dy)))
                
            # 检查强制邻居
            if (0 <= x + dx < len(self.grid) and 0 <= y - dy < len(self.grid[0]) and 
                self.grid[x][y - dy] == 1 and self.grid[x + dx][y - dy] == 0):
                print(f"添加强制邻居(对角线): ({x + dx}, {y - dy})")
                neighbors.append((x + dx, y - dy, (dx, -dy)))
            if (0 <= x - dx < len(self.grid) and 0 <= y + dy < len(self.grid[0]) and 
                self.grid[x - dx][y] == 1 and self.grid[x - dx][y + dy] == 0):
                print(f"添加强制邻居(对角线): ({x - dx}, {y + dy})")
                neighbors.append((x - dx, y + dy, (-dx, dy)))
                
        else:  # 直线移动
            print("处理直线移动")
            if dx != 0:  # 水平移动
                if 0 <= x + dx < len(self.grid) and self.grid[x + dx][y] == 0:
                    print(f"添加水平邻居: ({x + dx}, {y})")
                    neighbors.append((x + dx, y, (dx, 0)))
                    # 检查对角线强制邻居
                    for ny in (y + 1, y - 1):
                        if 0 <= ny < len(self.grid[0]):
                            if self.grid[x][ny] == 1 and self.grid[x + dx][ny] == 0:
                                print(f"添加强制邻居(水平): ({x + dx}, {ny})")
                                neighbors.append((x + dx, ny, (dx, ny - y)))
            else:  # 垂直移动
                if 0 <= y + dy < len(self.grid[0]) and self.grid[x][y + dy] == 0:
                    print(f"添加垂直邻居: ({x}, {y + dy})")
                    neighbors.append((x, y + dy, (0, dy)))
                    # 检查对角线强制邻居
                    for nx in (x + 1, x - 1):
                        if 0 <= nx < len(self.grid):
                            if self.grid[nx][y] == 1 and self.grid[nx][y + dy] == 0:
                                print(f"添加强制邻居(垂直): ({nx}, {y + dy})")
                                neighbors.append((nx, y + dy, (nx - x, dy)))

        return neighbors

    def _jump(self, x, y, dx, dy, goal):
        nx, ny = x + dx, y + dy
        
        # 边界和障碍检查提前
        if not (0 <= nx < len(self.grid) and 0 <= ny < len(self.grid[0])) or self.grid[nx][ny] == 1:
            return None

        # 优先检查是否到达目标
        if (nx, ny) == (goal.x, goal.y):
            return (nx, ny)

        # 修正强制邻居检测条件
        if dx != 0 and dy != 0:  # 对角线移动
            # 检查是否可以继续对角线移动
            if (0 <= nx + dx < len(self.grid) and 0 <= ny + dy < len(self.grid[0]) and 
                self.grid[nx + dx][ny + dy] == 0 and 
                (self.grid[nx][ny + dy] == 0 or self.grid[nx + dx][ny] == 0)):
                return (nx, ny)
            
            # 检查水平和垂直方向
            if (self._jump(nx, ny, dx, 0, goal) is not None or 
                self._jump(nx, ny, 0, dy, goal) is not None):
                return (nx, ny)
        else:  # 直线移动
            if dx != 0:  # 水平移动
                # 检查垂直方向的障碍物和可通行区域
                if ((ny+1 < len(self.grid[0]) and self.grid[x][ny+1] == 1 and self.grid[nx][ny+1] == 0) or 
                    (ny-1 >= 0 and self.grid[x][ny-1] == 1 and self.grid[nx][ny-1] == 0)):
                    return (nx, ny)
            else:  # 垂直移动
                # 检查水平方向的障碍物和可通行区域
                if ((nx+1 < len(self.grid) and self.grid[nx+1][y] == 1 and self.grid[nx+1][ny] == 0) or 
                    (nx-1 >= 0 and self.grid[nx-1][y] == 1 and self.grid[nx-1][ny] == 0)):
                    return (nx, ny)

        # 递归跳跃
        return self._jump(nx, ny, dx, dy, goal)

if __name__ == "__main__":
    # # 测试用例1 - 简单的地图
    # grid1 = [
    #     [0, 1, 0],
    #     [0, 0, 0],
    #     [0, 1, 0]
    # ]
    # start1 = (0, 0)
    # end1 = (2, 2)
    
    # print("测试用例1:")
    # jps_instance = JPS(grid1)
    # path1 = jps_instance.find_path(start1, end1)
    # print("找到的路径：", path1)
    
    # if path1:
    #     # 创建可视化地图
    #     visual_map = [row[:] for row in grid1]
    #     for x, y in path1:
    #         visual_map[x][y] = '*'
        
    #     # 打印地图
    #     print("\n地图可视化 (0:空地, 1:障碍物, *:路径):")
    #     for row in visual_map:
    #         print(" ".join(str(cell) if cell != '*' else '*' for cell in row))
    
    # print("\n" + "="*50 + "\n")
    
    # 测试用例2 - 稍复杂的地图
    grid2 = [
        [0, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1, 0]
    ]
    start2 = (0, 0)
    end2 = (3, 3)
    
    print("测试用例2:")
    jps_instance = JPS(grid2)
    path2 = jps_instance.find_path(start2, end2)
    print("找到的路径：", path2)
    
    if path2:
        # 创建可视化地图
        visual_map = [row[:] for row in grid2]
        for x, y in path2:
            visual_map[x][y] = '*'
        
        # 打印地图
        print("\n地图可视化 (0:空地, 1:障碍物, *:路径):")
        for row in visual_map:
            print(" ".join(str(cell) if cell != '*' else '*' for cell in row))
