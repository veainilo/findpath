class SimplePathFinder:
    def __init__(self, grid):
        """
        初始化路径规划器
        :param grid: 二维数组，0表示可通行，1表示障碍
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.visited = set()
        self.max_steps = 500  # 防止无限递归

    def find_path(self, start, end):
        """
        主路径查找方法
        :param start: 起始坐标 (x, y)
        :param end: 终点坐标 (x, y)
        :return: 路径列表 或 None（无解）
        """
        path = [start]
        current = start
        self.visited = {start}

        for _ in range(self.max_steps):
            if current == end:
                return path  # 到达终点

            # 尝试直线通行
            if self._direct_path_clear(current, end):
                path.append(end)
                return path

            # 寻找障碍物碰撞点
            collision = self._find_first_collision(current, end)
            if not collision:
                return path + [end]  # 意外情况，直接返回当前路径

            # 获取绕行点
            detour_points = self._find_detour_points(collision, current)
            if not detour_points:
                return None  # 无路可走

            # 选择更接近终点的绕行点
            current = min(detour_points, 
                         key=lambda p: self._heuristic(p, end))
            path.append(current)
            self.visited.add(current)

        return None  # 超过最大步数

    def _direct_path_clear(self, a, b):
        """检查直线是否可通行"""
        return all(self.grid[x][y] != 1 for (x, y) in self._bresenham_line(a, b))

    def _find_first_collision(self, start, end):
        """沿直线找到第一个障碍物坐标"""
        for point in self._bresenham_line(start, end):
            x, y = point
            if self.grid[x][y] == 1:
                return (x, y)
        return None

    def _find_detour_points(self, collision, from_point):
        """获取障碍物两侧的可通行点"""
        cx, cy = collision
        candidates = []
        
        # 检查8个相邻方向
        directions = [(-1,0), (1,0), (0,-1), (0,1),
                     (-1,-1), (-1,1), (1,-1), (1,1)]
        
        for dx, dy in directions:
            x, y = cx + dx, cy + dy
            if (0 <= x < self.rows and 0 <= y < self.cols and
                self.grid[x][y] == 0 and (x, y) not in self.visited):
                candidates.append((x, y))
        
        return candidates[:2]  # 返回前两个有效点

    def _heuristic(self, a, b):
        """估算两点距离"""
        return abs(a[0]-b[0]) + abs(a[1]-b[1])  # 曼哈顿距离

    def _bresenham_line(self, start, end):
        """生成两点间直线经过的格子坐标"""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steep = dy > dx
        
        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1
            dx, dy = dy, dx
            
        if x0 > x1:
            x0, x1 = x1, x0
            y0, y1 = y1, y0
            
        error = 0
        y_step = 1 if y0 < y1 else -1
        y = y0
        points = []
        
        for x in range(x0, x1 + 1):
            coord = (y, x) if steep else (x, y)
            points.append(coord)
            error += dy
            if 2 * error >= dx:
                y += y_step
                error -= dx
        return points

# 测试用例
if __name__ == "__main__":
    # 测试环境1：无障碍
    grid1 = [[0]*10 for _ in range(10)]
    finder1 = SimplePathFinder(grid1)
    print(finder1.find_path((0,0), (9,9)))  # 应返回直线路径
    
    # 测试环境2：单个障碍
    grid2 = [[0]*10 for _ in range(10)]
    grid2[5][3:7] = [1,1,1,1]
    finder2 = SimplePathFinder(grid2)
    print(finder2.find_path((0,0), (9,9)))  # 应绕行障碍
    
    # 测试环境3：封闭环境
    grid3 = [[1 if 0<x<9 and 0<y<9 else 0 for y in range(10)] for x in range(10)]
    finder3 = SimplePathFinder(grid3)
    print(finder3.find_path((0,0), (9,9)))  # 应返回None