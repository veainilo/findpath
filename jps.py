import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import colors
import heapq
import time
import random

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

    def jump(self, x, y, dx, dy, goal):
        self.jump_calls += 1
        nx, ny = x + dx, y + dy
        if not (0 <= nx < self.height and 0 <= ny < self.width) or self.grid[nx][ny] == 1:
            return None

        if (nx, ny) == (goal.x, goal.y):
            return (nx, ny)

        # 修正后的强制邻居判断（添加边界检查）
        if dx != 0 and dy != 0:  # 对角线移动
            # 检查水平方向和垂直方向是否有障碍物（添加边界检查）
            if (self.grid[x + dx][y] == 1 and self.grid[x][y + dy] == 0) or \
               (self.grid[x][y + dy] == 1 and self.grid[x + dx][y] == 0):
                return (nx, ny)
            
            # 继续对角线跳跃的同时检查直线方向（添加边界检查）
            if (0 <= nx + dx < self.height and self.jump(nx, ny, dx, 0, goal)) or \
               (0 <= ny + dy < self.width and self.jump(nx, ny, 0, dy, goal)):
                return (nx, ny)
        else:
            if dx != 0:  # 水平移动
                # 添加x和y方向的边界检查
                if ((y + 1 < self.width and 
                     x - dx >= 0 and x - dx < self.height and  # 新增x方向检查
                     self.grid[x][y + 1] == 0 and 
                     self.grid[x - dx][y + 1] == 1) or
                    (y - 1 >= 0 and 
                     x - dx >= 0 and x - dx < self.height and  # 新增x方向检查
                     self.grid[x][y - 1] == 0 and 
                     self.grid[x - dx][y - 1] == 1)):
                    return (nx, ny)
            else:  # 垂直移动
                # 添加x和y方向的边界检查
                if dx == 0:
                    # 检查上方方向（当dy=1时，y-dy可能为-1）
                    if (x + 1 < self.height and 
                        y - dy >= 0 and y - dy < self.width and  # 确保y-dy在有效范围内
                        self.grid[x + 1][y] == 0 and 
                        self.grid[x + 1][y - dy] == 1):
                        return (nx, ny)
                    
                    # 检查下方方向（当dy=-1时，y-dy可能超过宽度）
                    if (x - 1 >= 0 and 
                        y - dy >= 0 and y - dy < self.width and  # 确保y-dy在有效范围内
                        self.grid[x - 1][y] == 0 and 
                        self.grid[x - 1][y - dy] == 1):
                        return (nx, ny)

        # 优化递归方向
        if dx != 0 and dy != 0:
            return self.jump(nx, ny, dx, dy, goal)
        return self.jump(nx, ny, dx, dy, goal)

    def find_path(self, start, end):
        start_time = time.time()
        self.nodes_explored = 0
        self.jump_calls = 0
        
        open_list = []
        start_node = self.Node(*start)
        end_node = self.Node(*end)
        heapq.heappush(open_list, start_node)
        
        closed_list = set()
        
        while open_list:
            current = heapq.heappop(open_list)
            self.nodes_explored += 1

            if current.x == end_node.x and current.y == end_node.y:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                self.execution_time = time.time() - start_time
                return path[::-1]

            closed_list.add((current.x, current.y))
            
            neighbors = []
            for dx, dy in self.movements:
                jump_point = self.jump(current.x, current.y, dx, dy, end_node)
                if jump_point:
                    # 添加路径成本计算修正
                    step_x = abs(jump_point[0] - current.x)
                    step_y = abs(jump_point[1] - current.y)
                    cost = 10 * (step_x + step_y) + 4 * min(step_x, step_y)
                    neighbors.append((jump_point, cost))

            for neighbor, cost in neighbors:
                nx, ny = neighbor
                if (nx, ny) in closed_list:
                    continue
                
                new_node = self.Node(nx, ny, current)
                new_node.g = current.g + cost  # 使用修正后的成本计算
                new_node.h = self.heuristic(new_node, end_node)
                new_node.f = new_node.g + new_node.h

                heapq.heappush(open_list, new_node)

        self.execution_time = time.time() - start_time
        return None

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
                return path[::-1]

            if (current.x, current.y) in closed_dict:
                continue
            closed_dict[(current.x, current.y)] = current

            for dx, dy in self.movements:
                nx = current.x + dx
                ny = current.y + dy
                if 0 <= nx < self.height and 0 <= ny < self.width:
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

def visualize(grid, path=None, open_list=None, current=None, ax=None):
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
        mpatches.Patch(color='blue', label='路径'),
        mpatches.Patch(color='yellow', label='探索区域'),
        mpatches.Patch(color='cyan', label='当前节点')
    ]
    ax.legend(handles=legend, bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    return ax

# 添加随机障碍物生成函数
def generate_random_grid(size, obstacle_prob=0.2, start=(0,0), end=None):
    """生成包含随机障碍物的网格
    参数:
        size: 网格尺寸 (width, height)
        obstacle_prob: 障碍物生成概率 (0-1)
        start: 起点坐标 (确保可通行)
        end: 终点坐标 (自动设置为右下角并确保可通行)
    """
    width, height = size
    end = end or (height-1, width-1)
    
    grid = []
    for i in range(height):
        row = []
        for j in range(width):
            # 确保起点和终点可通行
            if (i,j) == start or (i,j) == end:
                row.append(0)
            else:
                row.append(1 if random.random() < obstacle_prob else 0)
        grid.append(row)
    return grid

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
        ("小型随机地图", 
         generate_random_grid(TEST_CONFIG['small_map'][:2], TEST_CONFIG['small_map'][2]),
         (0,0), (TEST_CONFIG['small_map'][0]-1, TEST_CONFIG['small_map'][1]-1)),
        
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

    # 更新测试输出表头
    print(f"{'测试场景':<15} | {'算法':<6} | {'时间(ms)':<8} | {'探索节点':<8} | {'路径存在':<8}")
    print("-"*65)
    
    for name, grid, start, end in test_cases:
        # 测试JPS
        jps = JPS(grid)
        jps_path = jps.find_path(start, end)
        print(f"{name:<15} | {'JPS':<6} | {round(jps.execution_time*1000,2):<8} | "
              f"{jps.nodes_explored:<8} | {'是' if jps_path else '否':<8}")
        
        # 测试A*
        astar = AStar(grid)
        astar_path = astar.find_path(start, end)
        print(f"{name:<15} | {'A*':<6} | {round(astar.execution_time*1000,2):<8} | "
              f"{astar.nodes_explored:<8} | {'是' if astar_path else '否':<8}")
        print("-"*65)

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
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12,6))
    visualize(random_grid, jps_path, ax=ax1)
    ax1.set_title(f"JPS 路径长度: {len(jps_path) if jps_path else '无'}")
    visualize(random_grid, astar_path, ax=ax2)
    ax2.set_title(f"A* 路径长度: {len(astar_path) if astar_path else '无'}")
    plt.show()
