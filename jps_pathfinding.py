import heapq
from math import sqrt

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

def heuristic(a, b):
    # 使用对角线距离（允许8方向移动时更准确）
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return 10 * (dx + dy) + (14 - 2 * 10) * min(dx, dy)

def get_neighbors(current, grid, end_node):
    # 在近距离检测中添加对角线路径检查
    if (abs(end_node.x - current.x) <= 2 or abs(end_node.y - current.y) <= 2) and \
       grid[end_node.x][end_node.y] == 0:
        # 修改为Bresenham直线算法检查路径
        dx = end_node.x - current.x
        dy = end_node.y - current.y
        steps = max(abs(dx), abs(dy))
        for i in range(steps + 1):
            x = current.x + round(i*dx/steps)
            y = current.y + round(i*dy/steps)
            if grid[x][y] == 1:
                break
        else:
            return [(end_node.x, end_node.y, (dx, dy))]

    # 根据父节点确定搜索方向
    neighbors = []
    x, y = current.x, current.y

    # 调试信息
    print(f"检查节点 ({x}, {y}) 的邻居")

    # 检查是否可以直接到达目标点
    if abs(end_node.x - x) <= 1 and abs(end_node.y - y) <= 1:
        if grid[end_node.x][end_node.y] == 0:
            dx = end_node.x - x
            dy = end_node.y - y
            if dx != 0 and dy != 0:  # 对角线移动到目标
                if grid[x][end_node.y] == 0 and grid[end_node.x][y] == 0:
                    print(f"添加目标点邻居: ({end_node.x}, {end_node.y})")
                    neighbors.append((end_node.x, end_node.y, (dx, dy)))
            else:  # 直线移动到目标
                print(f"添加目标点邻居: ({end_node.x}, {end_node.y})")
                neighbors.append((end_node.x, end_node.y, (dx, dy)))

    if not current.parent:  # 起点检查所有方向
        print("处理起点")
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
                    # 对角线移动时检查两个相邻格子是否可通行
                    if dx != 0 and dy != 0:
                        if grid[x][ny] == 1 or grid[nx][y] == 1:
                            continue
                    print(f"添加起点邻居: ({nx}, {ny})")
                    neighbors.append((nx, ny, (dx, dy)))
        return neighbors

    # 计算相对父节点的方向
    dx = current.x - current.parent.x
    dy = current.y - current.parent.y
    dx = 1 if dx > 0 else (-1 if dx < 0 else 0)
    dy = 1 if dy > 0 else (-1 if dy < 0 else 0)

    print(f"移动方向: dx={dx}, dy={dy}")

    # 自然邻居（沿原方向）
    if dx != 0 and dy != 0:  # 对角线移动
        print("处理对角线移动")
        # 检查对角线方向是否可行
        if (0 <= x + dx < len(grid) and 0 <= y + dy < len(grid[0]) and 
            grid[x + dx][y + dy] == 0 and grid[x][y + dy] == 0 and grid[x + dx][y] == 0):
            print(f"添加对角线邻居: ({x + dx}, {y + dy})")
            neighbors.append((x + dx, y + dy, (dx, dy)))
            
        # 检查水平和垂直方向
        if 0 <= x + dx < len(grid) and grid[x + dx][y] == 0:
            print(f"添加水平邻居: ({x + dx}, {y})")
            neighbors.append((x + dx, y, (dx, 0)))
        if 0 <= y + dy < len(grid[0]) and grid[x][y + dy] == 0:
            print(f"添加垂直邻居: ({x}, {y + dy})")
            neighbors.append((x, y + dy, (0, dy)))
            
        # 检查强制邻居
        if (0 <= x + dx < len(grid) and 0 <= y - dy < len(grid[0]) and 
            grid[x][y - dy] == 1 and grid[x + dx][y - dy] == 0):
            print(f"添加强制邻居(对角线): ({x + dx}, {y - dy})")
            neighbors.append((x + dx, y - dy, (dx, -dy)))
        if (0 <= x - dx < len(grid) and 0 <= y + dy < len(grid[0]) and 
            grid[x - dx][y] == 1 and grid[x - dx][y + dy] == 0):
            print(f"添加强制邻居(对角线): ({x - dx}, {y + dy})")
            neighbors.append((x - dx, y + dy, (-dx, dy)))
            
    else:  # 直线移动
        print("处理直线移动")
        if dx != 0:  # 水平移动
            if 0 <= x + dx < len(grid) and grid[x + dx][y] == 0:
                print(f"添加水平邻居: ({x + dx}, {y})")
                neighbors.append((x + dx, y, (dx, 0)))
                # 检查对角线强制邻居
                for ny in (y + 1, y - 1):
                    if 0 <= ny < len(grid[0]):
                        if grid[x][ny] == 1 and grid[x + dx][ny] == 0:
                            print(f"添加强制邻居(水平): ({x + dx}, {ny})")
                            neighbors.append((x + dx, ny, (dx, ny - y)))
        else:  # 垂直移动
            if 0 <= y + dy < len(grid[0]) and grid[x][y + dy] == 0:
                print(f"添加垂直邻居: ({x}, {y + dy})")
                neighbors.append((x, y + dy, (0, dy)))
                # 检查对角线强制邻居
                for nx in (x + 1, x - 1):
                    if 0 <= nx < len(grid):
                        if grid[nx][y] == 1 and grid[nx][y + dy] == 0:
                            print(f"添加强制邻居(垂直): ({nx}, {y + dy})")
                            neighbors.append((nx, y + dy, (nx - x, dy)))

    return neighbors

def get_straight_neighbors(node, grid, dx, dy):
    x, y = node.x, node.y
    neighbors = []
    # 自然直线方向
    if 0 <= x + dx < len(grid) and 0 <= y + dy < len(grid[0]):
        if grid[x+dx][y+dy] == 0:
            neighbors.append((x+dx, y+dy, (dx, dy)))
    # 可能的强制邻居（直线移动时的侧向检查）
    for side_dir in [(dy, dx), (-dy, -dx)]:
        nx, ny = x + side_dir[0], y + side_dir[1]
        if (0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 1):
            forced_nx, forced_ny = x + dx + side_dir[0], y + dy + side_dir[1]
            if (0 <= forced_nx < len(grid) and 0 <= forced_ny < len(grid[0])) and grid[forced_nx][forced_ny] == 0:
                neighbors.append((x+dx, y+dy, (dx, dy)))
    return neighbors

def get_diagonal_neighbors(node, grid, dx, dy):
    x, y = node.x, node.y
    neighbors = []
    # 自然对角线方向
    if grid[x+dx][y] == 0 or grid[x][y+dy] == 0:
        if 0 <= x + dx < len(grid) and 0 <= y + dy < len(grid[0]):
            if grid[x+dx][y+dy] == 0:
                neighbors.append((x+dx, y+dy, (dx, dy)))
    # 水平/垂直分量方向
    if grid[x+dx][y] == 0:
        neighbors.append((x+dx, y, (dx, 0)))
    if grid[x][y+dy] == 0:
        neighbors.append((x, y+dy, (0, dy)))
    return neighbors

def jump(grid, x, y, dx, dy, goal):
    nx, ny = x + dx, y + dy
    
    # 边界和障碍检查提前
    if not (0 <= nx < len(grid) and 0 <= ny < len(grid[0])) or grid[nx][ny] == 1:
        return None

    # 优先检查是否到达目标
    if (nx, ny) == (goal.x, goal.y):
        return (nx, ny)

    # 修改强制邻居检测逻辑
    if dx != 0 and dy != 0:  # 对角线移动
        # 添加对目标点的对角线直达检查
        if (abs(goal.x - nx) == abs(goal.y - ny)) and grid[goal.x][goal.y] == 0:
            if all(grid[x][y] == 0 for x, y in zip(range(nx, goal.x, dx), range(ny, goal.y, dy))):
                return (goal.x, goal.y)
                
        # 简化强制邻居判断
        if (grid[nx - dx][ny] == 1 and grid[nx - dx][ny + dy] == 0) or \
           (grid[nx][ny - dy] == 1 and grid[nx + dx][ny - dy] == 0):
            return (nx, ny)
            
        # 添加递归跳跃检查
        if jump(grid, nx, ny, dx, 0, goal) or jump(grid, nx, ny, 0, dy, goal):
            return (nx, ny)
    else:  # 直线移动
        # 优化直线跳跃的终点检测
        if (dx != 0 and ny == goal.y) or (dy != 0 and nx == goal.x):
            # 修改为逐步检查直线路径
            while (x, y) != (goal.x, goal.y):
                x += dx
                y += dy
                if not (0 <= x < len(grid) and 0 <= y < len(grid[0])) or grid[x][y] == 1:
                    return None
            return (goal.x, goal.y)
        
        # 添加直线移动的强制邻居检查
        if dx != 0:  # 水平移动
            if (y+1 < len(grid[0]) and grid[x][y+1] == 1 and grid[x+dx][y+1] == 0) or \
               (y-1 >= 0 and grid[x][y-1] == 1 and grid[x+dx][y-1] == 0):
                return (nx, ny)
        else:  # 垂直移动
            if (x+1 < len(grid) and grid[x+1][y] == 1 and grid[x+1][y+dy] == 0) or \
               (x-1 >= 0 and grid[x-1][y] == 1 and grid[x-1][y+dy] == 0):
                return (nx, ny)

    return jump(grid, nx, ny, dx, dy, goal)

def jps(start, end, grid):
    # 初始化开放列表和关闭集合
    open_list = []
    closed_set = set()

    start_node = Node(*start)
    end_node = Node(*end)
    start_node.h = heuristic(start_node, end_node)
    start_node.f = start_node.h

    heapq.heappush(open_list, start_node)
    
    while open_list:
        current = heapq.heappop(open_list)
        
        # 调试信息
        print(f"当前节点: ({current.x}, {current.y}), f={current.f}, g={current.g}, h={current.h}")
        
        if (current.x, current.y) == (end_node.x, end_node.y):
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        closed_set.add((current.x, current.y))
        
        # 获取邻居节点
        neighbors = get_neighbors(current, grid, end_node)
        print(f"邻居节点: {neighbors}")
        
        for nx, ny, direction in neighbors:
            if (nx, ny) in closed_set:
                continue
                
            dx, dy = direction
            
            # 如果邻居是目标点，直接添加到开放列表
            if (nx, ny) == (end_node.x, end_node.y):
                jump_point = (nx, ny)
            else:
                jump_point = jump(grid, current.x, current.y, dx, dy, end_node)
            
            if not jump_point:
                continue

            jx, jy = jump_point
            if grid[jx][jy] == 1:
                continue

            # 创建跳点节点
            jump_node = Node(jx, jy, current)
            
            # 计算实际成本
            if dx != 0 and dy != 0:
                jump_node.g = current.g + sqrt((jx-current.x)**2 + (jy-current.y)**2) * 14  # 对角线移动成本
            else:
                jump_node.g = current.g + (abs(jx-current.x) + abs(jy-current.y)) * 10  # 直线移动成本
                
            jump_node.h = heuristic(jump_node, end_node)
            jump_node.f = jump_node.g + jump_node.h

            # 检查开放列表中是否存在更优路径
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
                print(f"添加跳点: ({jx}, {jy}), f={jump_node.f}, g={jump_node.g}, h={jump_node.h}")

    return None  # 无路径

if __name__ == "__main__":
    # 测试用例1 - 简单的地图
    grid1 = [
        [0, 0, 0],
        [0, 1, 0],
        [0, 0, 0]
    ]
    start1 = (0, 0)
    end1 = (2, 2)
    
    print("测试用例1:")
    path1 = jps(start1, end1, grid1)
    print("找到的路径：", path1)
    
    if path1:
        # 创建可视化地图
        visual_map = [row[:] for row in grid1]
        for x, y in path1:
            visual_map[x][y] = '*'
        
        # 打印地图
        print("\n地图可视化 (0:空地, 1:障碍物, *:路径):")
        for row in visual_map:
            print(" ".join(str(cell) if cell != '*' else '*' for cell in row))
    
    print("\n" + "="*50 + "\n")
    
    # 测试用例2 - 稍复杂的地图
    grid2 = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    start2 = (0, 0)
    end2 = (4, 4)
    
    print("测试用例2:")
    path2 = jps(start2, end2, grid2)
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