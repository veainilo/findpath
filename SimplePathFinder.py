import matplotlib.pyplot as plt
import numpy as np

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
        """检查直线是否可通行（增加端点检查）"""
        line_points = self._bresenham_line(a, b)
        # 排除起点（因为起点可能位于障碍边缘）
        return all(self.grid[x][y] != 1 for (x, y) in line_points[1:-1])

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
        
        # 优化方向顺序，优先水平/垂直方向
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 优先四方向
            (-1,-1), (-1,1), (1,-1), (1,1)     # 次要对角线方向
        ]
        
        # 根据行进方向优化优先级
        dx = cx - from_point[0]
        dy = cy - from_point[1]
        if dx != 0:  # 垂直方向运动优先
            directions.insert(0, (0, 1 if dy > 0 else -1))
        if dy != 0:  # 水平方向运动优先
            directions.insert(1, (1 if dx > 0 else -1, 0))
        
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

    # 新增可视化方法
    def plot_path(self, path, title='Path Visualization'):
        """
        可视化路径和障碍物
        :param path: 路径列表
        :param title: 图表标题
        """
        plt.figure(figsize=(8, 8))
        
        # 转置网格以适应坐标系
        grid_array = np.array(self.grid).T
        plt.imshow(grid_array, cmap='Greys', origin='lower', 
                  vmin=0, vmax=1, alpha=0.3)
        
        # 绘制路径
        if path:
            x_coords = [p[0] for p in path]
            y_coords = [p[1] for p in path]
            plt.plot(x_coords, y_coords, 'r-', linewidth=2)
            plt.scatter(x_coords, y_coords, c='red', s=40)
            
        # 标记起点和终点
        if path:
            start = path[0]
            end = path[-1]
            plt.scatter(start[0], start[1], c='green', s=100, marker='s', label='Start')
            plt.scatter(end[0], end[1], c='blue', s=100, marker='*', label='End')
        
        # 增强障碍物显示
        obstacle_x, obstacle_y = np.where(np.array(self.grid).T == 1)
        plt.scatter(obstacle_x, obstacle_y, c='black', s=100, marker='s', alpha=0.5, label='Obstacles')
        
        # 添加坐标标注
        for x in range(self.rows):
            for y in range(self.cols):
                plt.text(x, y, f'({x},{y})', fontsize=8, ha='center', va='center', alpha=0.5)
        
        plt.title(title)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True, which='both', color='lightgray', linestyle='--')
        plt.legend()
        plt.show()

# 修改测试用例部分
if __name__ == "__main__":
    # 测试环境1：无障碍
    grid1 = [[0]*10 for _ in range(10)]
    finder1 = SimplePathFinder(grid1)
    path1 = finder1.find_path((0,0), (9,9))
    print("Test 1 Path:", path1)
    finder1.plot_path(path1, '无障碍路径')
    
    # 测试环境2：单个障碍
    grid2 = [[0]*10 for _ in range(10)]
    grid2[5][3:7] = [1,1,1,1]
    finder2 = SimplePathFinder(grid2)
    path2 = finder2.find_path((0,0), (9,9))
    print("\nTest 2 Path:", path2)
    finder2.plot_path(path2, '障碍绕行路径')
    
    # 测试环境3：封闭环境
    grid3 = [[1 if 0<x<9 and 0<y<9 else 0 for y in range(10)] for x in range(10)]
    finder3 = SimplePathFinder(grid3)
    path3 = finder3.find_path((0,0), (9,9))
    print("\nTest 3 Result:", path3)
    finder3.plot_path(path3, '无解情况')